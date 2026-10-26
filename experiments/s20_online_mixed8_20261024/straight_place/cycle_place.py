"""Persistent-runtime one-box placement, frozen release, empty observation return.

Caller retains the Runtime, pickup logic, source visibility and occupancy list.
No SDK, hardware, replay, runtime construction or filesystem writes on import.
"""
from dataclasses import asdict
import time
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
import newton
from compact_prior import CompactPrior
from compare import Box, p

UNIFORM20 = (.55, .40, .25)


def place_and_return(r, fitted, contact_tcp, occupied, move, taught_pose,
                     publish, report, realtime=False):
    """Plan ALL loaded and independent empty segments, then commit, return Box.

    Exact orchestrator signature. Raises RuntimeError on exhausted search;
    failed planning never advances the persistent runtime or adds occupancy.
    `move` is accepted for API compatibility; commit uses prechecked joint
    samples directly so no callback replanning can cause a partial cycle.
    `contact_tcp` is the recorded suction TCP pose at this item's contact.
    Return occupancy is the actual released OBB expressed conservatively as an
    axis-aligned Box in ULD coordinates, using the declared uniform20 size prior.
    """
    began = time.perf_counter()
    held = r.w.held
    if held is None:
        raise ValueError('place_and_return requires an active held actor')
    label = r.model.body_label[held]
    source_index = r.boxes.index(held)
    q0 = np.array(r.q).copy()
    initial_poses = r.state.body_q.numpy().copy()
    saved_held = r.w.held
    saved_source = r.collision.current_source
    saved_arrived = set(r.collision.arrived)
    actual_offset = np.array(r.w.grip_offset).copy()
    actual_rotation = r.w.grip_rot
    contact_tcp = np.asarray(contact_tcp, dtype=float)
    measured_rotation = R.from_matrix(np.array(fitted['T_world_box'])[:3,:3])
    # Uniform dimensions are a declared TASK PRIOR, never read from the actor.
    estimated_center = np.array(fitted['top_center_world_m'])-measured_rotation.apply([0.,0.,UNIFORM20[2]/2])
    contact_rotation = R.from_quat(contact_tcp[3:])
    estimated_offset = contact_rotation.inv().apply(estimated_center-contact_tcp[:3])
    estimated_rotation = contact_rotation.inv()*measured_rotation
    h = r.hull
    tf = next(v for v in r.cfg['static_transforms'] if v['child']=='container_link')
    cr = R.from_euler('xyz',tf['rotation_rpy']); translation = np.array(tf['translation'])
    origin = np.array([h.door_x,h.inner_y0,h.floor_z])
    item = dict(id=label, completed=False, phase='planning', events=[], failures=[],
        uniform_size_prior_m=list(UNIFORM20), observed_dimensions_m=list(fitted['dimensions_m']),
        size_source='Explicit known identical20in task prior; observed top center/orientation supply pose and estimated attachment',
        estimated_grip_offset_m=estimated_offset.tolist(),estimated_grip_rotation_xyzw=estimated_rotation.as_quat().tolist(),
        simulated_actual_grip_offset_m=actual_offset.tolist(),simulated_actual_grip_rotation_xyzw=actual_rotation.as_quat().tolist(),
        geometric_side_clearance_m=.02,final_release_offset_m=.005,
        collision='Original CAD/FCL every20ms; initial separation uses original suction/source exemption only, strict empty route after verified clearance',
        scope='Ideal rigid suction and frozen actual release; no contact dynamics or hardware',
        committed_prefix=len(occupied),motion_callback_replanning=False)
    report.setdefault('placements',[]).append(item)
    shared_events = report.setdefault('events',[])
    plans = []
    publish()

    def failed(context, error):
        item['failures'].append(dict(**context,**error))
        item['planning_progress']=dict(**context,last_failure_stage=error.get('stage'),failures=len(item['failures']))
        # Publish only the physical current state, never a hypothetical release.
        restore_planning_context()
        publish()
        print(f"{label} placement search: candidate={context.get('rank')} phase={error.get('stage')} reason={error.get('reason')} pairs={error.get('pairs',[])}",flush=True)

    def cad(q):
        if r.validate(q):
            return None
        return [list(pair) for pair in r.collision.last_hits]

    def contact_details():
        import fcl
        result=[]
        for pair in r.collision.last_hits:
            indices=[r.model.shape_label.index(name) for name in pair]
            contacts=fcl.CollisionResult()
            fcl.collide(r.collision.objs[indices[0]],r.collision.objs[indices[1]],
                fcl.CollisionRequest(num_max_contacts=8,enable_contact=True),contacts)
            result.append(dict(pair=list(pair),bodies=[r.model.body_label[r.collision.bodies[i]] if r.collision.bodies[i]>=0 else 'world' for i in indices],
                max_depth_m=max((float(c.penetration_depth) for c in contacts.contacts),default=None),
                normals_world=[c.normal.tolist() for c in contacts.contacts]))
        return result

    def pose_for_q(q):
        r.w.fk.joint_q.assign(np.asarray(q))
        newton.eval_fk(r.model,r.w.fk.joint_q,r.w.fk.joint_qd,r.w.fk)
        poses = initial_poses.copy()
        poses[:r.w.arm_count] = r.w.fk.body_q.numpy()[:r.w.arm_count]
        tcp = poses[r.tcp]; rot = R.from_quat(tcp[3:])
        poses[held] = np.r_[tcp[:3]+rot.apply(actual_offset),(rot*actual_rotation).as_quat()]
        return poses

    def segment(qstart, position, quaternion, name):
        start = time.perf_counter()
        r.ik.model.joint_q.assign(np.asarray(qstart))
        newton.eval_fk(r.ik.model,r.ik.model.joint_q,r.ik.model.joint_qd,r.ik.model)
        tcp = r.ik.model.body_q.numpy()[r.ik.contact_body].copy()
        position = np.array(position); quaternion = np.array(quaternion)
        duration = max(float(np.linalg.norm(position-tcp[:3]))/.30,
                       float((R.from_quat(tcp[3:]).inv()*R.from_quat(quaternion)).magnitude()),.02)
        samples = max(2,int(np.ceil(duration/.02)))
        rot = Slerp([0.,1.],R.from_quat([tcp[3:],quaternion])); q = np.array(qstart).copy(); path=[]
        for index, alpha in enumerate(np.linspace(0.,1.,samples+1)[1:]):
            target = tcp[:3]*(1-alpha)+position*alpha
            ok,q = r.ik._solve_once(target,rot(alpha).as_quat(),q,check_collision=False)
            if not ok:
                return None,dict(stage=name,reason='native_IK_unsolved',sample=index,
                    target_world_m=target.tolist(),diagnostics=dict(r.ik.last_diagnostics),q_rad=q.tolist())
            pairs = cad(q)
            if pairs:
                return None,dict(stage=name,reason='original_CAD_rejected',sample=index,pairs=pairs,
                    contact_details=contact_details(),q_rad=q.tolist())
            path.append(q.copy())
        event = dict(stage=name,planning_wall_s=time.perf_counter()-start,reference_s=len(path)*.02,frames=len(path))
        return (name,np.array(path),event),None

    def chain(q, goals):
        result=[]
        for name,position,quaternion in goals:
            planned,error = segment(q,position,quaternion,name)
            if error:return None,error
            result.append(planned);q=planned[1][-1]
        return result,None

    def restore_planning_context():
        r.state.body_q.assign(initial_poses)
        r.w.held=saved_held;r.collision.current_source=saved_source
        r.collision.arrived=set(saved_arrived)

    try:
        if cad(q0):
            raise RuntimeError(f'{label}: held-entry state already CAD-rejected: {r.collision.last_hits}')
        entry_tcp = initial_poses[r.tcp].copy()
        lower,error = segment(q0,entry_tcp[:3]+[0,0,-.05],entry_tcp[3:],label+'_ENTRY_LOWER_5CM')
        if error:
            item['failures'].append(error);base_q=q0.copy();entry_prefix=[]
        else:
            entry_prefix=[lower];base_q=lower[1][-1]
        prior=CompactPrior(r.cfg,h,max_box_top_m=1.,depth_step_m=.1)
        prior.horizontal_first=True;prior.row_first=True;prior.contact_first=False
        # Task-level clearance, identical for all items; no actor-pose correction.
        prior.lateral_clearance_m=.02
        candidates,stats=prior.candidates(UNIFORM20,list(occupied))
        item['candidate_stats']=stats
        observation=np.array(taught_pose('observation'))
        taught_lift=np.array(taught_pose('post_pick_lift'))
        taught_entry=np.array(taught_pose('container_entry'))
        selected=None
        for rank,b in enumerate(candidates):
            item['planning_progress']=dict(rank=rank,candidates=len(candidates),failures=len(item['failures']))
            publish()
            yaw=-np.pi/2 if tuple(b.dims)==(UNIFORM20[1],UNIFORM20[0],UNIFORM20[2]) else 0.
            heights=(0.,max([.4]+[v.pos[2]+v.dims[2]+.05 for v in occupied]))
            routes=[]
            for height in heights:
                route=p.route_to(b,list(occupied),h,min_entry_z=height)
                if route is not None and route not in routes:routes.append(route)
            initial_rotation=R.from_quat(entry_tcp[3:])
            headings=sorted((0.,np.pi),key=lambda a:(initial_rotation.inv()*cr*R.from_euler('z',yaw+a)*estimated_rotation.inv()).magnitude())
            for heading in headings:
                tool_rotation=cr*R.from_euler('z',yaw+heading)*estimated_rotation.inv()
                def target(point):
                    center=translation+cr.apply(np.array(point)+np.array(b.dims)/2+origin)
                    return center-tool_rotation.apply(estimated_offset),tool_rotation.as_quat()
                for route_index,route in enumerate(routes):
                    # Cross the pedestal at the taught entry height before
                    # descending near the doorway, rather than over the base.
                    route=[list(point) for point in route]
                    route[0][0]=max(route[0][0],-.10)
                    restore_planning_context()
                    entry_pose=pose_for_q(base_q)[r.tcp]
                    goals=[(label+'_ALIGN_OUTSIDE',entry_pose[:3],tool_rotation.as_quat())]
                    door_position,door_quaternion=target(route[0])
                    high_door=door_position.copy()
                    high_door[2]=max(high_door[2],entry_pose[2])
                    goals.append((label+'_CLEAR_BASE_AT_ENTRY_HEIGHT',high_door,door_quaternion))
                    for leg,point in enumerate(route):
                        point=np.array(point).copy()
                        if leg==len(route)-1:point[2]+=.005
                        position,quaternion=target(point)
                        goals.append((f'{label}_PLACE_LEG{leg}',position,quaternion))
                    loaded,error=chain(base_q,goals)
                    failure_context=dict(rank=rank,heading_deg=float(np.degrees(heading)),route_index=route_index)
                    if error:
                        failed(failure_context,error);continue
                    release_q=loaded[-1][1][-1];release_poses=pose_for_q(release_q)
                    release_pose=release_poses[held].copy();release_tcp=release_poses[r.tcp].copy()
                    # Independent empty simulation context: actual frozen release
                    # obstacle, no held-body or source-contact exemptions.
                    r.state.body_q.assign(release_poses);r.w.held=None
                    r.collision.current_source=None;r.collision.arrived=saved_arrived|{source_index}
                    initial_contact=cad(release_q)
                    release_contacts=contact_details() if initial_contact else []
                    empty=None;last_error=None
                    variants=[('direct',[-.05,0.,.03]),('direct',[-.05,0.,.05])]
                    variants += [('lifted_reverse',[0.,0.,z]) for z in (.03,.05,.10,.15)]
                    for withdrawal,separation_delta in variants:
                        lift=separation_delta[2]
                        # Only the original suction/source contact exemption
                        # remains during detachment, never arm/fixture contact.
                        r.collision.current_source=source_index
                        separation,last_error=segment(release_q,release_tcp[:3]+cr.apply(separation_delta),
                            release_tcp[3:],label+'_EMPTY_SEPARATE')
                        r.collision.current_source=None
                        if separation is None:
                            item.setdefault('empty_variant_failures',[]).append(dict(**failure_context,withdrawal=withdrawal,
                                separation_delta_container_m=separation_delta,**last_error))
                            continue
                        separated_q=separation[1][-1]
                        remaining_contact=cad(separated_q)
                        if remaining_contact:
                            last_error=dict(stage=label+'_SEPARATION_CLEARANCE',reason='strict_CAD_rejected_after_separation',
                                pairs=remaining_contact,contact_details=contact_details(),q_rad=separated_q.tolist())
                            item.setdefault('empty_variant_failures',[]).append(dict(**failure_context,withdrawal=withdrawal,
                                separation_delta_container_m=separation_delta,**last_error))
                            continue
                        empty_goals=[]
                        if withdrawal=='lifted_reverse':
                            for leg,point in enumerate(reversed(route[:-1])):
                                position,_=target(point)
                                position=position+cr.apply([0.,0.,lift])
                                empty_goals.append((f'{label}_EMPTY_EXIT{leg}',position,release_tcp[3:]))
                        # Primary path is a single checked Cartesian translation
                        # to taught entry, retaining release heading until outside.
                        empty_goals.extend([(label+'_EMPTY_DIRECT_ENTRY_POSITION',taught_entry[:3],release_tcp[3:]),
                            (label+'_EMPTY_ENTRY_HEADING',taught_entry[:3],taught_entry[3:]),
                            (label+'_EMPTY_TAUGHT_LIFT',taught_lift[:3],taught_lift[3:]),
                            (label+'_RETURN_OBSERVATION',observation[:3],observation[3:])])
                        remainder,last_error=chain(separated_q,empty_goals)
                        if remainder is not None:
                            empty=[separation]+remainder;break
                        item.setdefault('empty_variant_failures',[]).append(dict(**failure_context,withdrawal=withdrawal,
                            separation_delta_container_m=separation_delta,**last_error))
                    if empty is None:
                        failed(dict(**failure_context,phase='empty_return'),last_error);continue
                    selected=dict(box=b,rank=rank,heading=heading,route=route,loaded=entry_prefix+loaded,
                                  empty=empty,release_q=release_q,release_pose=release_pose,
                                  initial_release_contacts=release_contacts,separation_lift_m=lift,
                                  withdrawal=withdrawal,separation_delta_container_m=separation_delta)
                    break
                if selected:break
            if selected:break
        restore_planning_context()
        if selected is None:
            raise RuntimeError(f'{label}: no complete loaded-place + independent empty-return plan; {len(item["failures"])} retained failures')
        item.update(phase='committing',selected_candidate=selected['rank'],nominal=asdict(selected['box']),
            heading_deg=float(np.degrees(selected['heading'])),route=selected['route'],
            all_segments_planned_before_commit=True,
            initial_release_contacts=selected['initial_release_contacts'],
            separation_lift_m=selected['separation_lift_m'],
            withdrawal=selected['withdrawal'],
            separation_delta_container_m=selected['separation_delta_container_m'],
            initial_separation_uses_original_suction_source_exemption=True,
            initial_vertical_separation_uses_original_suction_source_exemption=selected['separation_delta_container_m'][0]==0.,
            empty_route_after_separation_checked_without_source_exemption=True,
            strict_clearance_verified_at_end_of_separation=True)
        publish()
        def commit(segments):
            for name,path,event in segments:
                if name==label+'_EMPTY_SEPARATE':r.collision.current_source=source_index
                execution_start=time.perf_counter()
                for index,q in enumerate(path):
                    r.frame(q,name)
                    if realtime and (index+1)%10==0:publish()
                    if realtime:time.sleep(max(0.,execution_start+(index+1)*.02-time.perf_counter()))
                committed_event=dict(event,execution_wall_s=time.perf_counter()-execution_start)
                item['events'].append(committed_event);shared_events.append(committed_event);publish()
                if name==label+'_EMPTY_SEPARATE':r.collision.current_source=None
        commit(selected['loaded'])
        release=r.state.body_q.numpy()[held].copy()
        r.fixed[held]=release;r.w.held=None;r.collision.current_source=None
        r.collision.arrived=saved_arrived|{source_index}
        r.frame(r.q,label+'_RELEASE_ACTUAL_FROZEN');publish()
        commit(selected['empty'])
        center=cr.inv().apply(release[:3]-translation)-origin
        extent=abs((cr.inv()*R.from_quat(release[3:])).as_matrix())@np.array(UNIFORM20)
        result=Box(label,tuple(float(v) for v in extent),tuple(float(v) for v in center-extent/2))
        item.update(completed=True,phase='returned_observation',actual_frozen_pose_world=release.tolist(),
            occupied=asdict(result),occupancy_source='Actual frozen released OBB -> conservative ULD AABB; dimensions are declared uniform20 prior',
            wall_s=time.perf_counter()-began,empty_return_completed=True,
            nominal_hull_side_outside_m=max(0.,-result.pos[1]),nominal_hull_floor_outside_m=max(0.,-result.pos[2]))
        publish()
        return result
    except Exception as exc:
        if item['phase']=='planning':restore_planning_context()
        item.update(error=str(exc),wall_s=time.perf_counter()-began)
        publish()
        raise
