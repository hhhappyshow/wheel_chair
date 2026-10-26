"""Persistent process-worker planning; no simulator/GPU initialization on import.

Use ProcessPoolExecutor(max_workers=1, mp_context=get_context('spawn'),
    initializer=initialize_worker, initargs=(out, catalog, waypoints)).
Submit plan_in_worker(request) immediately after the model/grasp is known,
continue picking/lifting/transferring, then await and validate at actual entry.
Worker reference success never authorizes application to the main Runtime.
"""
from copy import deepcopy
from dataclasses import asdict
import os
from pathlib import Path
import sys
import time

_WORKER = None


def estimate_attachments(fitted, planned_contact_tcp):
    """Observed-data estimates only. Returns target and ground-prior transforms.

    Target estimate matches cycle_place unchanged. Predicted actor uses observed
    ground + known .25m height and ground-projected observed long-axis yaw.
    """
    import numpy as np
    from scipy.spatial.transform import Rotation as R
    contact = np.asarray(planned_contact_tcp, dtype=float)
    if contact.shape != (7,):
        raise ValueError('planned_contact_tcp must be world xyz + xyzw, length7')
    top = np.asarray(fitted['top_center_world_m'], dtype=float)
    observed_rotation = R.from_matrix(np.asarray(fitted['T_world_box'], dtype=float)[:3,:3])
    contact_rotation = R.from_quat(contact[3:])
    target_center = top-observed_rotation.apply([0.,0.,.125])
    target_offset = contact_rotation.inv().apply(target_center-contact[:3])
    target_rotation = contact_rotation.inv()*observed_rotation
    ground = fitted.get('ground_plane')
    if ground is not None:
        plane = np.asarray(ground, dtype=float)
        plane = plane/np.linalg.norm(plane[:3])
        if plane[2] < 0:
            plane = -plane
        normal = plane[:3]
        foot = top-normal*(top@normal+plane[3])
        predicted_center = foot+normal*.125
        long_axis = observed_rotation.apply([1.,0.,0.])
        long_axis = long_axis-normal*(long_axis@normal)
        long_axis = long_axis/np.linalg.norm(long_axis)
        predicted_rotation_world = R.from_matrix(np.column_stack((long_axis,np.cross(normal,long_axis),normal)))
        physical_source = 'Observed ground plane + known uniform20 half-height; observed long axis projected onto ground; no GT pose or actual grip'
    else:
        predicted_center = np.asarray(fitted['T_world_box'],dtype=float)[:3,3]
        predicted_rotation_world = observed_rotation
        physical_source = 'Ground plane absent: observed fitted center/orientation fallback; no GT pose or actual grip'
    predicted_offset = contact_rotation.inv().apply(predicted_center-contact[:3])
    predicted_rotation = contact_rotation.inv()*predicted_rotation_world
    return dict(
        target_estimated_grip=dict(offset_m=target_offset.tolist(),rotation_xyzw=target_rotation.as_quat().tolist(),
                                  source='Observed top center minus known half-height along fitted top normal; unchanged serial target convention'),
        predicted_physical_grip=dict(offset_m=predicted_offset.tolist(),rotation_xyzw=predicted_rotation.as_quat().tolist(),
                                    source=physical_source,source_center_world_m=predicted_center.tolist(),
                                    source_rotation_world_xyzw=predicted_rotation_world.as_quat().tolist()),
        known_uniform20_dimensions_m=[.55,.40,.25],actual_grip_used=False)


def initialize_worker(out, catalog, waypoints):
    """ProcessPool initializer: construct ONE persistent, separate Runtime.

    `catalog` is the declared all-uniform20 scene/task prior (not future arrivals).
    `waypoints` is the configuration dict with observation/post_pick_lift/
    container_entry entries, each containing q_rad. Inherited Warp cache is shared.
    Call only in a spawned worker, never by forking an initialized CUDA process.
    """
    global _WORKER
    if _WORKER is not None:
        raise RuntimeError('Worker is already initialized')
    began = time.perf_counter()
    sys.dont_write_bytecode = True
    out = Path(out).resolve();out.mkdir(parents=True,exist_ok=True)
    os.environ.setdefault('WARP_CACHE_PATH',str(Path.home()/'.cache'/'warp'))
    for key,folder in (('XDG_CACHE_HOME','cache'),('MPLCONFIGDIR','mpl_config')):
        os.environ[key] = str(out/folder)
    os.environ.setdefault('PYOPENGL_USE_ACCELERATE','0')
    for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
        os.environ[key] = '1'
    parent = Path(__file__).resolve().parent.parent
    sys.path[:0] = [str(parent),str(parent/'perception')]
    import numpy as np
    import warp as wp
    wp.config.kernel_cache_dir = os.environ['WARP_CACHE_PATH']
    from perception.suitcase_capture import make_runtime
    from straight_place.cycle_place import place_and_return
    from compare import Box
    sizes = [list(map(float,size)) for size in catalog]
    if not sizes or not all(np.allclose(size,[.55,.40,.25],rtol=0,atol=1e-9) for size in sizes):
        raise ValueError('This worker implements the declared uniform20 task only')
    runtime = make_runtime(out,sizes)
    _WORKER = dict(runtime=runtime,helper=place_and_return,Box=Box,
                   waypoints=deepcopy(waypoints),catalog=sizes,requests=0,
                   initialization_s=time.perf_counter()-began,pid=os.getpid())


def worker_ready():
    """Initializer readiness probe; does not claim to precompile every IK kernel."""
    if _WORKER is None:
        raise RuntimeError('Worker initializer has not completed')
    return dict(ok=True,initialized=True,pid=_WORKER['pid'],
                worker_pid=_WORKER['pid'],initialization_s=_WORKER['initialization_s'],
                ik_kernels_fully_prewarmed=False)


def plan_in_worker(request):
    """Plan one current item and hypothetical empty return in worker only.

    Required request keys:
      fitted, planned_contact_tcp (world xyz+xyzw7), entry_q (6),
      occupied (Box iterable or list of {id,dims,pos}),
      placed_world_poses ({'B1': world_pose7, ...}, exact committed prefix),
      source_index (zero based), scene_version (opaque serializable revision).
    Also accepts planned_contact_tcp7/entry_q6 and zero-based string prefix keys.
    Optional: request_id, dispatched_at. No future items or actual grip.

    Returns success/error, joint_q/stages/times, body_labels, release_frame_index,
    predicted_initial_entry_q, estimated_grip + BOTH named attachment estimates,
    placement_meta, worker start/end/compute timings and unchanged scene_version.
    Main MUST validate revision, actual initial q, actual attachment and frozen
    release/empty trajectory before applying; otherwise replan serially.
    """
    if _WORKER is None:
        raise RuntimeError('Call initialize_worker as the process initializer first')
    import numpy as np
    import newton
    from scipy.spatial.transform import Rotation as R
    began = time.perf_counter();began_unix = time.time()
    worker = _WORKER;r = worker['runtime'];worker['requests'] += 1
    response = dict(success=False,scene_version=deepcopy(request.get('scene_version')),
        request_id=deepcopy(request.get('request_id')),source_index=request.get('source_index'),
        worker_pid=worker['pid'],worker_request_number=worker['requests'],
        worker_initialization_s=worker['initialization_s'],worker_start_monotonic_s=began,
        worker_start_unix_s=began_unix,dispatched_monotonic_s=request.get('dispatched_monotonic_s',request.get('dispatched_at')),
        dispatched_at=request.get('dispatched_at',request.get('dispatched_monotonic_s')),
        hypothetical_worker_reference=True,apply_requires_actual_revalidation=True,
        main_runtime_mutated=False,future_arrivals_used=0)
    local_report = dict(placements=[],events=[],scope='Hypothetical estimated-attachment process-worker reference; not main simulation execution')
    try:
        source_index = int(request['source_index'])
        if source_index < 0 or source_index >= len(r.boxes):
            raise ValueError('source_index outside uniform scene catalog')
        entry_q = np.asarray(request['entry_q'] if 'entry_q' in request else request['entry_q6'],dtype=float)
        if entry_q.shape != (6,):
            raise ValueError('entry_q must contain six joints')
        fit = deepcopy(request['fitted'])
        contact = np.asarray(request['planned_contact_tcp'] if 'planned_contact_tcp' in request else request['planned_contact_tcp7'],dtype=float)
        attachments = estimate_attachments(fit,contact)
        predicted = attachments['predicted_physical_grip']
        occupancy=[]
        for box in request['occupied']:
            values = box if isinstance(box,dict) else asdict(box)
            occupancy.append(worker['Box'](values['id'],tuple(values['dims']),tuple(values['pos'])))
        expected = {f'B{i+1}' for i in range(source_index)}
        fixed = dict(request['placed_world_poses'])
        if set(fixed) == {str(i) for i in range(source_index)}:
            fixed = {f'B{int(index)+1}':pose for index,pose in fixed.items()}
        if set(fixed) != expected or {box.id for box in occupancy} != expected:
            raise ValueError('Provide exactly the committed current prefix, without future/current actor poses')
        r.history.clear();r.qs.clear();r.stages.clear();r.simtime.clear();r.t=0.
        r.fixed={body:np.array([100.,100.,100.,0.,0.,0.,1.]) for body in r.boxes}
        for label,pose in fixed.items():
            body = r.model.body_label.index(label)
            r.fixed[body] = np.asarray(pose,dtype=float).copy()
        r.q=entry_q.copy();r.w.q[:6]=entry_q
        r.w.held=r.boxes[source_index]
        r.w.grip_offset=np.array(predicted['offset_m'])
        r.w.grip_rot=R.from_quat(predicted['rotation_xyzw'])
        r.collision.arrived=set(range(source_index+1));r.collision.current_source=source_index
        r.frame(entry_q,'ASYNC_PREDICTED_HELD_ENTRY')
        initial_entry = r.state.body_q.numpy()[r.tcp].copy()
        predicted_actor = r.state.body_q.numpy()[r.w.held].copy()
        def taught_pose(name):
            q = np.asarray(worker['waypoints'][name]['q_rad'],dtype=float)
            r.ik.model.joint_q.assign(q)
            newton.eval_fk(r.ik.model,r.ik.model.joint_q,r.ik.model.joint_qd,r.ik.model)
            return r.ik.model.body_q.numpy()[r.ik.contact_body].copy()
        def no_replanning(*args,**kwargs):
            raise RuntimeError('Worker helper unexpectedly requested callback replanning')
        result = worker['helper'](r,fit,contact,occupancy,no_replanning,taught_pose,
                                  lambda:None,local_report,realtime=False)
        release_indices=[i for i,stage in enumerate(r.stages) if stage.endswith('_RELEASE_ACTUAL_FROZEN')]
        if len(release_indices) != 1:
            raise RuntimeError('Expected exactly one hypothetical release event')
        placement = deepcopy(local_report['placements'][-1])
        placement['worker_attachment_semantics']='simulated_actual_grip fields describe predicted observed-ground-prior attachment in this worker, NOT actual main-runtime grip'
        response.update(success=True,predicted_initial_entry_q=entry_q.tolist(),
            predicted_initial_entry_tcp=initial_entry.tolist(),predicted_initial_actor_world_pose=predicted_actor.tolist(),
            estimated_grip=deepcopy(predicted),attachments=attachments,
            joint_q=np.asarray(r.qs).copy(),stages=list(r.stages),times=np.asarray(r.simtime).copy(),
            body_labels=list(r.model.body_label),release_frame_index=release_indices[0],
            predicted_release_world_pose=r.history[release_indices[0]][r.boxes[source_index]].tolist(),
            predicted_occupied=asdict(result),placement_meta=placement,events=deepcopy(local_report['events']),
            committed_prefix_world_poses=deepcopy(fixed),
            contact_transition='Only the initial separation segment uses original suction/source exemption; strict empty clearance afterward',
            estimated_reference_validated_only=True)
    except Exception as exc:
        response.update(error=f'{type(exc).__name__}: {exc}',placement_meta=deepcopy(local_report['placements'][-1]) if local_report['placements'] else None,
                        events=deepcopy(local_report['events']))
    finally:
        ended = time.perf_counter()
        response.update(worker_end_monotonic_s=ended,worker_end_unix_s=time.time(),compute_s=ended-began,
            compute_includes_failed_candidates=True,queue_and_initialization_excluded_from_compute=True)
        response.update(ok=response['success'],placement=deepcopy(response.get('placement_meta')),
            worker_started_at=began,worker_finished_at=ended,worker_compute_s=ended-began)
    return response
