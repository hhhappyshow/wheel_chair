"""Fresh, stage-scoped ServoJ commissioning evidence and tracking geometry.

An empty hardware probe establishes startup, timing and normal-stop behavior.
It is not a carrying-cycle qualification. Every trial still needs a new
whole-robot/scene/self-collision proof including its tracking envelope.
"""
import copy
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
from continuous_reference import plan_digest
from native_execution_reference import compile_reference, validated_blocks
from manual_session import file_hash, save
PROBE = 'servoj_probe_1791453780232847625'
PERIOD_S = 0.02
LOOKAHEAD_S = 0.2
SENIOR_LOOKAHEAD_S = 0.1

def startup_evidence(root, model_hash):
    path = Path(root) / 'runs/manual' / __import__('os').environ.get('S20_SERVO_STARTUP_PROBE', PROBE) / 'report.json'
    report = json.loads(path.read_text())
    geometry = json.loads(Path(report['geometry_file']).read_text())
    feedback = report.get('feedback', [])
    final = report.get('final', {})
    if report.get('backend') != 'servoj' or report.get('completed') is not True or report.get('motion_sent') is not True or (len(feedback) < 100) or (report.get('period_s') != PERIOD_S) or (report.get('lookahead_s') != LOOKAHEAD_S) or (geometry.get('robot_model_sha256') != model_hash) or (final.get('fsm') != 33) or any(final.get('io', {'unknown': 1}).values()) or (np.max(np.abs(final['qd'])) > math.radians(0.05)) or (time.time() - path.stat().st_mtime > 24.0 * 3600.0):
        raise RuntimeError('Current-model physical ServoJ startup/stop evidence required')
    jitter = max((float(row['deadline_lateness_s']) for row in feedback))
    rpc = max((float(row['send_rpc_s']) for row in feedback))
    return {'file': str(path), 'sha256': file_hash(path), 'geometry_sha256': file_hash(report['geometry_file']), 'controller_version': report['controller_version'], 'maximum_observed_jitter_s': jitter, 'maximum_observed_send_rpc_s': rpc, 'scope': 'empty_startup_timing_normal_stop_only', 'loaded_cycle_previously_qualified': False}

def _cup_bounds(model):
    from native_motion_geometry import body_box_bounds
    from plan_contact_trial import mesh_separation_vertices
    output = {}
    for body in model.bodies:
        if body.link != 'suction_panel':
            continue
        vertices = mesh_separation_vertices(model, body)
        bounds = (vertices.min(axis=0), vertices.max(axis=0)) if vertices is not None else body_box_bounds(model, body)
        if bounds is None:
            raise ValueError('Cup contact geometry unavailable')
        output[body.name] = bounds
    return output

def stage_geometry(plan, state, root, run):
    """Recreate the same observed world and attachment; never open the SDK."""
    from ament_index_python.packages import get_package_share_directory
    from articulated_svsdf import ArticulatedModel
    from manual_motion import Planner, add_box, matrix, world_from_rows
    from run_articulated_svsdf import resolve_package
    xml = Path(state['urdf']).read_text()
    srdf = ET.parse(Path(get_package_share_directory('luggage_description')) / 'config/S20_with_camera.srdf')
    disabled = [(row.get('link1'), row.get('link2')) for row in srdf.findall('disable_collisions')]
    source_file = Path(run) / 'pick_plan.json'
    contact = json.loads(source_file.read_text()) if source_file.is_file() else None
    phases = {row['phase'] for row in plan['segments']}
    if plan.get('scene') and (plan.get('loaded') or phases.intersection(('release_retreat', 'withdraw', 'return_observe'))):
        raise ValueError('Attached/released stages require authenticated attachment reconstruction')
    if contact is None and state.get('contact_plan') and (not plan.get('scene')):
        if file_hash(state['contact_plan']) != state.get('contact_plan_sha256'):
            raise RuntimeError('Contact geometry changed since grasp')
        contact = json.loads(Path(state['contact_plan']).read_text())
    if plan.get('scene') or (contact is not None and (not plan.get('loaded')) and (not state.get('transfer_plan'))):
        source = plan if plan.get('scene') else contact
        model = ArticulatedModel(xml, resolve_package)
        fast = ArticulatedModel(xml, resolve_package, collision_mode='primitive_proxy')
        world = world_from_rows(source['scene'])
        fixed = {body.link for body in model.bodies if not model.ancestors[body.name]}
        allowed = [(link, name) for link in fixed for name in world.ids if name in ('floor', 'pedestal_prior')]
        contacts = []
        fast_contacts = []
        if contact is not None and source is contact:
            target = next((row for row in source['scene'] if row['name'] == 'target'))
            (pose, half) = (np.asarray(target['pose']), np.asarray(target['size']) / 2.0)
            world.target_top = float(pose[2, 3] + np.abs(pose[2, :3]) @ half)
            world.cup_bounds = _cup_bounds(model)
            world.cup_bounds.update(_cup_bounds(fast))
            contacts = [(body.name, 'target', -(0.02 + contact['proxy_surface_above_contact_m'])) for body in model.bodies if body.link == 'suction_panel']
            fast_contacts = [(body.name, 'target', -(0.02 + contact['proxy_surface_above_contact_m'])) for body in fast.bodies if body.link == 'suction_panel']
        tilt = plan.get('initialization_seed', {}).get('whole_path_tilt_limit_rad', math.radians(20.0))
        return ({phase: (model, fast, world, contacts, fast_contacts, allowed, float(tilt)) for phase in dict.fromkeys((row['phase'] for row in plan['segments']))}, disabled, None)
    if contact is None or not state.get('selected_id'):
        raise ValueError('Current attachment geometry required for this stage')
    config = json.loads((Path(root) / 'config/manual_cell.json').read_text())
    if file_hash(Path(root) / 'config/manual_cell.json') != state.get('contact_scene_file_sha256'):
        raise RuntimeError('Fixed scene changed since contact')
    case = state['items'][state['selected_id']]['box']
    planner = Planner(xml, config, contact['scene'], case, state['contact_q'], allow_moveit_seed=False)
    try:
        placed = [dict(item['box'], id=key) for (key, item) in state['items'].items() if item['status'].startswith('PLACED') and key != state['selected_id']]
        parent = plan
        if not plan.get('loaded'):
            parent = json.loads(Path(state['transfer_plan']).read_text())
            if file_hash(state['transfer_plan']) != state.get('transfer_plan_sha256'):
                raise RuntimeError('Release/return source changed')
        planner.add_placed(placed)
        if parent.get('candidate'):
            world = planner.placement_world(parent['candidate'], placed)
        else:
            world = planner.world
        if not plan.get('loaded'):
            settled = matrix(parent['settled_box_pose'])
            add_box(world, 'released_target', parent.get('placed_collision_size_m', case['size_m']), settled)
            world.released_top = float(planner.cell[:3, 2] @ settled[:3, 3] + case['size_m'][2] / 2.0)
        model = planner.loaded if plan.get('loaded') else planner.free
        contexts = {}
        for phase in dict.fromkeys((row['phase'] for row in plan['segments'])):
            source_world = planner.world if plan.get('loaded') and phase == 'lift' else world
            phase_world = world_from_rows(copy.deepcopy(source_world.rows))
            if hasattr(world, 'released_top'):
                phase_world.released_top = world.released_top
            (fast, contacts, fast_contacts, allowed) = planner._collision_policy(model, phase_world, phase)
            contexts[phase] = (model, fast, phase_world, contacts, fast_contacts, allowed, math.radians(20.0))
        return (contexts, planner.disabled, planner)
    except BaseException:
        planner.close()
        raise

def _path_box(path, left, right):
    midpoint = path.evaluate((left + right) / 2.0)
    excursion = path.excursion(left, right)
    return (midpoint - excursion, midpoint + excursion)

def _tracking_limits(path, left, right):
    start = max(0.0, left - LOOKAHEAD_S - PERIOD_S)
    end = min(float(path.times[-1]), right + PERIOD_S)
    peak = np.zeros(path.positions.shape[1])
    for (piece, (a, b)) in enumerate(zip(path.times[:-1], path.times[1:])):
        if b < start or a > end:
            continue
        dt = b - a
        (low, high) = ((max(start, a) - a) / dt, (min(end, b) - a) / dt)
        for axis in range(len(peak)):
            polynomial = np.polynomial.polynomial.polyder(path.coefficients[piece][:, axis])
            inputs = [low, high] + [root for root in path.extrema_roots(piece, axis, 1) if low <= root <= high]
            peak[axis] = max(peak[axis], float(np.max(np.abs(np.polynomial.polynomial.polyval(inputs, polynomial) / dt))))
    monitored = peak * (LOOKAHEAD_S + PERIOD_S) + math.radians(0.05)
    reserve = peak * PERIOD_S + math.radians(0.05)
    return (monitored, reserve)

def certify_tracking(plan, reference, contexts, disabled, *, deadline, block_indices=None):
    from articulated_svsdf import QuinticJointPath, SweptClearance
    from native_motion_geometry import certify_motion_box
    from point_command_compaction import downward_joint_box
    from tracking_joint_subdivision import certify_failed_pairs, validate_cover
    (_, blocks) = validated_blocks(plan, reference)
    proof = {'schema': 's20_servo_tracking_tube/v1', 'source_plan_digest': plan_digest(plan), 'reference_digest': reference['reference_digest'], 'blocks': [], 'robot_model_sha256': reference['robot_model_sha256'], 'full_scene_and_self_collision_checked': False, 'loaded': bool(plan.get('loaded')), 'method': 'continuous_polynomial_range_plus_conservative_joint_tracking_tube', 'not_discrete_sample_collision_acceptance': True}
    for (block_index, (block, path)) in enumerate(blocks):
        if block_indices is not None and block_index not in block_indices:
            continue
        (model, fast, world, contacts, fast_contacts, allowed, tilt) = contexts[block['phase']]
        checker = SweptClearance(model, world, QuinticJointPath([0.0, 1.0], [path.positions[0]] * 2), disabled_self_pairs=disabled, allowed_touch_pairs=allowed, contact_pair_margins=contacts, margin=0.005, deadline_monotonic=deadline)
        proxy_checker = SweptClearance(fast, world, QuinticJointPath([0.0, 1.0], [path.positions[0]] * 2), disabled_self_pairs=disabled, allowed_touch_pairs=allowed, contact_pair_margins=fast_contacts, margin=0.005, deadline_monotonic=deadline)
        keys = {frozenset((body.name, target)): i for (i, (body, target, _, _)) in enumerate(checker._pairs)}
        queue = [(float(a), float(b), 0) for (a, b) in zip(path.times[:-1], path.times[1:])][::-1]
        (intervals, supplementary) = ([], [])
        while queue:
            if time.monotonic() >= deadline:
                raise TimeoutError('Fresh ServoJ tracking-tube proof deadline exhausted')
            (left, right, level) = queue.pop()
            (monitored_error, reserve) = _tracking_limits(path, left, right)
            padding = monitored_error + reserve
            (lower, upper) = _path_box(path, left, right)
            nominal_span = float(np.max(upper-lower))
            (lower, upper) = (lower - padding, upper + padding)
            if any((lower[i] < limits[0] or upper[i] > limits[1] for (i, limits) in enumerate(reference['joint_bounds_rad'].values()))):
                raise ValueError('ServoJ tracking tube exceeds joint position limits')
            attitude = downward_joint_box(model, lower, upper, max_tilt_rad=tilt)
            failures = ['attitude'] if not attitude else []
            if attitude:
                from plan_contact_trial import certify_box
                failures = certify_box(fast, world, lower, upper, disabled, fast_contacts, allowed, deadline, proxy_checker, padding_rad=0.0)
                if failures:
                    pairs = {frozenset((row['body'].split(':proxy')[0], row['target'].split(':proxy')[0])) for row in failures}
                    if not pairs.issubset(keys):
                        raise ValueError('ServoJ tracking proxy/exact pair mapping incomplete')
                    failures = certify_box(model, world, lower, upper, disabled, contacts, allowed, deadline, checker, pair_indices=[keys[key] for key in pairs], padding_rad=0.0)
                    if failures:
                        failures = certify_motion_box(model, world, lower, upper, disabled,
                            contacts, allowed, deadline, evidence=supplementary, failures=failures)
                    if failures and nominal_span <= math.radians(.4):
                        cover = certify_failed_pairs(model, world, lower, upper, disabled,
                            contacts, allowed, deadline, checker, failures)
                        if cover is not None:
                            validate_cover(cover)
                            supplementary.append(dict(cover, start_s=left, end_s=right))
                            failures = []
            if failures:
                if level >= 9 or right - left <= 0.002:
                    failure = ValueError('Unresolved ServoJ tracking tube: ' + str(failures[:2]))
                    failure.tracking_interval = dict(phase=block['phase'], start_s=left,
                        end_s=right, lower_rad=lower.tolist(), upper_rad=upper.tolist(),
                        failures=failures, supplemental_count=len(supplementary))
                    raise failure
                middle = (left + right) / 2.0
                queue.extend(((middle, right, level + 1), (left, middle, level + 1)))
                continue
            intervals.append({'start_s': left, 'end_s': right, 'lower_rad': lower.tolist(), 'upper_rad': upper.tolist(), 'monitored_error_rad': monitored_error.tolist(), 'guard_reserve_rad': reserve.tolist(), 'certified_extra_padding_rad': padding.tolist()})
        monitored_error = np.max([row['monitored_error_rad'] for row in intervals], axis=0)
        reserve = np.max([row['guard_reserve_rad'] for row in intervals], axis=0)
        proof['blocks'].append({'index': block_index, 'phase': block['phase'], 'duration_s': float(path.times[-1]), 'monitored_error_rad': monitored_error.tolist(), 'guard_reserve_rad': reserve.tolist(), 'certified_extra_padding_rad': (monitored_error + reserve).tolist(), 'intervals': intervals, 'tracking_padding_method': 'exact_local_velocity_extrema_with_lag_history', 'world_rows_digest': plan_digest(world.rows), 'world_rows': world.rows, 'supplementary_primitive_proofs': supplementary, 'whole_joint_tube_attitude_limit_rad': tilt, 'full_scene_and_self_collision_checked': True})
    proof['full_scene_and_self_collision_checked'] = len(proof['blocks']) == len(blocks)
    proof['proof_digest'] = plan_digest(proof)
    return proof

def _refresh_reference_digest(reference):
    reference['reference_duration_s'] = sum((b['duration_s'] for b in reference['blocks']))
    reference.pop('reference_digest', None)
    reference['reference_digest'] = plan_digest({k: v for (k, v) in reference.items() if k != 'preparation_s'})

def _limit_block_speed(block, bounds, ceiling):
    """Retime only this rest-to-rest phase, preserving its exact spatial curve."""
    from articulated_svsdf import QuinticJointPath
    limited = copy.deepcopy(block)
    data = limited['reference']
    peak = max(block['kinematic_profile']['max_abs_velocity'].values())
    scale = max(1.0, peak / math.radians(ceiling))
    data['times'] = (np.asarray(data['times']) * scale).tolist()
    data['qd'] = (np.asarray(data['qd']) / scale).tolist()
    data['qdd'] = (np.asarray(data['qdd']) / scale ** 2).tolist()
    limited['duration_s'] = float(data['times'][-1])
    limited['servo_trial_uniform_slowdown'] = scale
    limited['reference_velocity_ceiling_deg_s'] = ceiling
    limited['profile']['velocity_cap_deg_s'] = min(limited['profile']['velocity_cap_deg_s'], ceiling)
    path = QuinticJointPath(data['times'], data['q'], data['qd'], data['qdd'])
    limited['kinematic_profile'] = path.kinematic_profile(bounds, {name: math.radians(limited['profile']['velocity_cap_deg_s']) for name in bounds}, {name: math.radians(limited['profile']['acceleration_cap_deg_s2']) for name in bounds})
    if not limited['kinematic_profile']['kinematically_within_limits']:
        raise ValueError('Phase-local retiming exceeds the existing kinematic policy')
    return limited

def compile_trial_reference(plan, xml, *, speed_scale=3., velocity_cap_deg_s=30.,
                            contact_time_scale=1., phase_time_scales=None):
    from articulated_svsdf import QuinticJointPath
    if not math.isfinite(contact_time_scale) or not 1. <= contact_time_scale <= 5.:
        raise ValueError('Explicit contact duration multiplier must be within 1..5')
    reference = compile_reference(plan, xml, speed_scale=speed_scale,
                                  velocity_cap_deg_s=velocity_cap_deg_s)
    scales = dict(phase_time_scales or {})
    if any(not math.isfinite(value) or not 1. <= value <= 5. for value in scales.values()):
        raise ValueError('Explicit phase duration multipliers must be within 1..5')
    scales['normal_contact'] = contact_time_scale
    for block in reference['blocks']:
        scale = scales.get(block['phase'], 1.)
        if scale == 1.:
            continue
        data = block['reference']
        data['times'] = (np.asarray(data['times'])*scale).tolist()
        data['qd'] = (np.asarray(data['qd'])/scale).tolist()
        data['qdd'] = (np.asarray(data['qdd'])/scale**2).tolist()
        block['duration_s'] = float(data['times'][-1])
        block['explicit_phase_time_scale'] = scale
        path = QuinticJointPath(data['times'], data['q'], data['qd'], data['qdd'])
        bounds = reference['joint_bounds_rad']
        block['kinematic_profile'] = path.kinematic_profile(bounds,
            {name: math.radians(block['profile']['velocity_cap_deg_s']) for name in bounds},
            {name: math.radians(block['profile']['acceleration_cap_deg_s2']) for name in bounds})
    reference['explicit_contact_time_scale'] = contact_time_scale
    reference['explicit_phase_time_scales'] = scales
    _refresh_reference_digest(reference)
    return reference


def prepare_trial(plan, state, root, run, *, speed_scale=3.0, velocity_cap_deg_s=30.0,
                  empty_rehearsal_geometry=None, contact_time_scale=1., phase_time_scales=None):
    from execution_speed_policy import phase_speed_profile
    began = time.monotonic()
    evidence = startup_evidence(root, plan['robot_model_sha256'])
    if empty_rehearsal_geometry is None:
        (contexts, disabled, planner) = stage_geometry(plan, state, root, run)
    else:
        if plan.get('loaded') or plan.get('scope') != 'empty_rehearsal_actual_obstacles_retained':
            raise ValueError('Explicit empty rehearsal required for supplied geometry')
        contexts, disabled = empty_rehearsal_geometry
        planner = None
    attempts = []
    try:
        reference = compile_trial_reference(plan, Path(state['urdf']).read_text(),
            speed_scale=speed_scale, velocity_cap_deg_s=velocity_cap_deg_s,
            contact_time_scale=contact_time_scale, phase_time_scales=phase_time_scales)
        original_blocks = copy.deepcopy(reference['blocks'])
        certified = []
        for (index, original) in enumerate(original_blocks):
            profile = phase_speed_profile(original['phase'], bool(plan.get('loaded')), speed_scale, velocity_cap_deg_s)
            original['feedback_profile'] = profile
            maximum = profile['velocity_cap_deg_s']
            ceilings = [maximum]
            for ceiling in ceilings:
                reference['blocks'][index] = _limit_block_speed(original, reference['joint_bounds_rad'], ceiling)
                _refresh_reference_digest(reference)
                started = time.monotonic()
                try:
                    partial = certify_tracking(plan, reference, contexts, disabled, deadline=started + 20.0, block_indices={index})
                    certified.append(partial['blocks'][0])
                    attempts.append({'block': index, 'phase': original['phase'], 'reference_velocity_ceiling_deg_s': ceiling, 'tracking_proof_passed': True, 'proof_wall_s': time.monotonic() - started})
                    break
                except (ValueError, TimeoutError) as exc:
                    attempts.append({'block': index, 'phase': original['phase'], 'reference_velocity_ceiling_deg_s': ceiling, 'tracking_proof_passed': False, 'proof_wall_s': time.monotonic() - started, 'reason': str(exc), 'failed_interval': getattr(exc, 'tracking_interval', None)})
            else:
                save(Path(run) / 'servoj_trial_rejections.json', {'attempts': attempts, 'motion_sent': False})
                raise RuntimeError('REQUESTED_SPEED_NOT_VALIDATED: no automatic speed fallback: '
                    + original['phase'] + ': ' + attempts[-1]['reason'])
        _refresh_reference_digest(reference)
        proof = dict(partial, blocks=certified, reference_digest=reference['reference_digest'],
            full_scene_and_self_collision_checked=True, automatic_speed_fallback=False,
            timing_policy='requested_caps_no_hidden_whole_phase_slowdown')
        proof.pop('proof_digest', None)
        proof['proof_digest'] = plan_digest(proof)
    finally:
        if planner is not None:
            planner.close()
    proof_path = Path(run) / 'servoj_tracking_tube.json'
    save(proof_path, proof)
    configuration = {'schema': 's20_continuous_execution/v1', 'qualified': True, 'qualification_scope': 'this_stage_supervised_trial_not_full_cycle', 'backend': 'hrif_servoj', 'startup_mode': 'official_servoj', 'units': 'degrees', 'controller_host': '192.168.0.10', 'robot_model_sha256': plan['robot_model_sha256'], 'normal_stop_qualified': True, 'tracking_tube_qualified': True, 'qualification_evidence': {'startup': evidence, 'tracking': str(proof_path), 'tracking_sha256': file_hash(proof_path)}, 'reference_digest': reference['reference_digest'], 'source_plan_digest': plan_digest(plan), 'expires_at_s': time.time() + 120.0, 'period_s': PERIOD_S, 'lookahead_s': SENIOR_LOOKAHEAD_S, 'startup_parameter_source': 'senior_jetson_execute_servo_j_path_20ms_100ms', 'conservative_tracking_lookahead_bound_s': LOOKAHEAD_S, 'changed_lookahead_is_supervised_trial_not_prior_physical_qualification': True, 'feedback_period_s': 0.04, 'deadline_tolerance_s': 0.006, 'rpc_budget_s': 0.016, 'feedback_group_budget_s': PERIOD_S - 0.002, 'feedback_budget_scope': 'seven_state_reads_not_one_send_rpc', 'max_tracking_error_deg': min(1.0, math.degrees(max((max(b['monitored_error_rad']) for b in proof['blocks'])))), 'settle_timeout_s': 5.0, 'allowed_fsm': [25, 33], 'controller_parameters_written': False, 'full_cycle_physically_verified': False}
    configuration.update(timing_policy='wall_clock_no_replay_tracking_guarded',
        tracking_error_policy='stage_verified_local_joint_tube',
        global_fixed_tracking_guard_enabled=False,
        estimated_acceleration_is_diagnostic_only=True,
        maximum_update_gap_s=0.1, single_rpc_budgets_are_diagnostics_only=True,
        update_gap_basis='actual_lookahead_plus_gap_within_verified_220ms_tracking_lag')
    save(Path(run) / 'servoj_reference.json', reference)
    save(Path(run) / 'servoj_stage_trial_configuration.json', configuration)
    save(Path(run) / 'servoj_stage_trial_preparation.json', {'preparation_s': time.monotonic() - began, 'attempts': attempts, 'motion_sent': False, 'stage_scoped': True})
    return (reference, configuration, proof)

def validate_trial(plan, reference, configuration, proof):
    if configuration.get('qualification_scope') != 'this_stage_supervised_trial_not_full_cycle':
        raise ValueError('Explicit stage-scoped trial required')
    evidence = configuration['qualification_evidence']
    unsigned = dict(proof)
    digest = unsigned.pop('proof_digest', None)
    if digest != plan_digest(unsigned) or proof['source_plan_digest'] != plan_digest(plan) or proof['reference_digest'] != reference['reference_digest'] or (configuration['reference_digest'] != reference['reference_digest']) or (proof.get('full_scene_and_self_collision_checked') is not True) or (proof.get('loaded') != bool(plan.get('loaded'))) or (file_hash(evidence['tracking']) != evidence['tracking_sha256']) or (file_hash(evidence['startup']['file']) != evidence['startup']['sha256']):
        raise ValueError('Stage-scoped ServoJ proof/evidence changed')
    (_, blocks) = validated_blocks(plan, reference)
    if len(blocks) != len(proof['blocks']):
        raise ValueError('Missing ServoJ tracking blocks')
    for (index, ((block, path), tube)) in enumerate(zip(blocks, proof['blocks'])):
        if tube['index'] != index or tube['phase'] != block['phase']:
            raise ValueError('ServoJ tracking phases reordered')
        from tracking_joint_subdivision import validate_cover
        for cover in tube.get('supplementary_primitive_proofs', []):
            if cover.get('method') == 'complete_joint_box_binary_cover':
                validate_cover(cover)
        previous = 0.0
        for row in tube['intervals']:
            if abs(row['start_s'] - previous) > 1e-07 or row['end_s'] <= row['start_s']:
                raise ValueError('ServoJ tracking tube has a time gap')
            (lower, upper) = _path_box(path, row['start_s'], row['end_s'])
            (monitored, reserve) = _tracking_limits(path, row['start_s'], row['end_s'])
            if not np.allclose(row['monitored_error_rad'], monitored, rtol=0.0, atol=1e-10) or not np.allclose(row['guard_reserve_rad'], reserve, rtol=0.0, atol=1e-10):
                raise ValueError('ServoJ local tracking error bounds changed')
            padding = monitored + reserve
            if not np.allclose(row['certified_extra_padding_rad'], padding, rtol=0.0, atol=1e-10):
                raise ValueError('ServoJ local tracking guard changed')
            if np.any(lower - padding < np.asarray(row['lower_rad']) - 1e-07) or np.any(upper + padding > np.asarray(row['upper_rad']) + 1e-07):
                raise ValueError('ServoJ tracking curve changed since fresh geometry proof')
            previous = row['end_s']
        if abs(previous - path.times[-1]) > 1e-07:
            raise ValueError('ServoJ tracking proof does not cover the complete curve')
