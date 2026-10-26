"""Generic official Newton pose-batch IK; no scene/domain/history candidates.

One supplied seed per supplied target, 35 iterations, same objectives and
acceptance tolerances as _solve_once(check_collision=False). Always returns the
actual optimized q, including failed rows; caller owns propagation and CAD.
No Newton/Warp import or device initialization until a nonempty solve call.
"""
import time

import numpy as np


def solve(ik, positions, rotations, seeds):
    """Nx3 world xyz, Nx4 xyzw, Nx6 radians -> (list of answers, stats).

    Reuse the workspace to retain solver instances by batch size. First calls
    include possible JIT compilation; cold_call_s is NOT compile-only time.
    Call warmup with the intended shape before a measured planning interval.
    No model.joint_q/q_last mutation: a batch has no unique terminal state.
    """
    began = time.perf_counter()
    positions, rotations, seeds = [np.asarray(value, dtype=np.float32)
                                  for value in (positions, rotations, seeds)]
    if positions.size == rotations.size == seeds.size == 0:
        return [], dict(wall_s=time.perf_counter()-began, problems=0, converged=0,
                        iterations=35, cold_call_s=0., warm_call_s=0.)
    count = len(positions)
    if any(value.shape != (count, width) or not np.isfinite(value).all()
           for value, width in ((positions, 3), (rotations, 4), (seeds, 6))):
        raise ValueError('Expected finite positions Nx3, rotations Nx4 xyzw, seeds Nx6')
    if not np.allclose(np.linalg.norm(rotations, axis=1), 1., atol=1e-4, rtol=0):
        raise ValueError('Target xyzw quaternions must be unit length')
    import warp as wp
    from newton import ik as nik
    from newton_ik_planner import IK_ROTATION_TOLERANCE

    device = ik.model.joint_q.device
    if ik.model.joint_q.shape != (6,):
        raise ValueError('This adapter expects the six-coordinate S20 IK model')
    cache = getattr(ik, '_pose_batch_solvers', None)
    if cache is None:
        cache = {}
        ik._pose_batch_solvers = cache
    cold = count not in cache
    setup_began = time.perf_counter()
    if cold:
        target_positions = wp.array(positions, dtype=wp.vec3, device=device)
        target_rotations = wp.array(rotations, dtype=wp.vec4, device=device)
        objectives = [
            nik.IKObjectivePosition(link_index=ik.contact_body, link_offset=wp.vec3(0, 0, 0),
                                    target_positions=target_positions, weight=1.),
            nik.IKObjectiveRotation(link_index=ik.contact_body, link_offset_rotation=wp.quat_identity(),
                                    target_rotations=target_rotations, weight=.7),
            nik.IKObjectiveJointLimit(joint_limit_lower=ik.model.joint_limit_lower,
                                     joint_limit_upper=ik.model.joint_limit_upper, weight=10.)]
        solver = nik.IKSolver(model=ik.model, n_problems=count, objectives=objectives,
                              lambda_initial=.1, jacobian_mode=nik.IKJacobianType.ANALYTIC)
        cache[count] = (solver, target_positions, target_rotations)
    solver, target_positions, target_rotations = cache[count]
    target_positions.assign(positions)
    target_rotations.assign(rotations)
    q = wp.array(seeds.copy(), dtype=wp.float32, device=device)
    wp.synchronize_device(device)
    setup_s = time.perf_counter()-setup_began
    step_began = time.perf_counter()
    solver.step(q, q, iterations=35)
    wp.synchronize_device(device)
    step_s = time.perf_counter()-step_began
    solutions = q.numpy().copy()
    # IKSolver.step ends with compute_costs(final q), refreshing its batched FK.
    # These poses therefore correspond to returned q, not input seeds.
    actual = solver.body_q.numpy()[:, ik.contact_body].copy()
    position_error = np.linalg.norm(actual[:, :3]-positions, axis=1)
    rotation_error = np.array([ik._quat_error(pose[3:7], target)
                               for pose, target in zip(actual, rotations)])
    lower = ik.model.joint_limit_lower.numpy()[:6]
    upper = ik.model.joint_limit_upper.numpy()[:6]
    limits = ((solutions >= lower-1e-4) & (solutions <= upper+1e-4)).all(axis=1)
    good = ((position_error <= ik.position_tolerance_m)
            & (rotation_error <= IK_ROTATION_TOLERANCE) & limits)
    answers = [dict(ok=bool(good[i]), q=solutions[i].copy(), diagnostics=dict(
        position_error_m=float(position_error[i]), rotation_error_rad=float(rotation_error[i]),
        joint_limits=bool(limits[i]), collision_free=None, collision_checked=False,
        target_position_world_m=positions[i].tolist(), actual_position_world_m=actual[i, :3].tolist(),
        actual_rotation_xyzw=actual[i, 3:7].tolist())) for i in range(count)]
    wall = time.perf_counter()-began
    return answers, dict(wall_s=wall, problems=count, converged=int(good.sum()), iterations=35,
        setup_s=setup_s, step_and_sync_s=step_s, first_call_for_batch_shape=cold,
        cold_call_s=wall if cold else 0., warm_call_s=0. if cold else wall,
        compile_only_s=None, compilation_may_be_included=cold,
        position_tolerance_m=float(ik.position_tolerance_m),
        rotation_tolerance_rad=float(IK_ROTATION_TOLERANCE), joint_limit_tolerance_rad=1e-4,
        input_seeds_mutated=False, scalar_workspace_state_mutated=False,
        scope='Pose IK only; original CAD and loaded/empty paths remain caller checks; failures are not impossibility proofs')


def warmup(ik, positions, rotations, seeds):
    """Explicit real 35-iteration solve outside main timing; no hidden targets."""
    _, stats = solve(ik, positions, rotations, seeds)
    return dict(stats, warmup=True)
