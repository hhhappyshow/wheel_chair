"""Persistent-runtime one-box placement, frozen release, empty observation return.

Caller retains the Runtime, pickup logic, source visibility and occupancy list.
No SDK, hardware, replay, runtime construction or filesystem writes on import.
"""
from dataclasses import asdict
import time
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
import newton
from straight_place.sparse_prior import SparseCompactPrior
from scipy.interpolate import PchipInterpolator
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
    dimensions = tuple(getattr(r, 'planner_options', {}).get('current_dimensions_m', UNIFORM20))
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
    estimated_center = np.array(fitted['top_center_world_m'])-measured_rotation.apply([0.,0.,dimensions[2]/2])
    contact_rotation = R.from_quat(contact_tcp[3:])
    estimated_offset = contact_rotation.inv().apply(estimated_center-contact_tcp[:3])
    estimated_rotation = contact_rotation.inv()*measured_rotation
    h = r.hull
    tf = next(v for v in r.cfg['static_transforms'] if v['child']=='container_link')
    cr = R.from_euler('xyz',tf['rotation_rpy']); translation = np.array(tf['translation'])
    origin = np.array([h.door_x,h.inner_y0,h.floor_z])
    item = dict(id=label, completed=False, phase='planning', events=[], failures=[],
        planning_dimensions_m=list(dimensions), observed_dimensions_m=list(fitted['dimensions_m']),
        size_source=fitted.get('planning_size_source','Explicit uniform20 task prior'),
        estimated_grip_offset_m=estimated_offset.tolist(),estimated_grip_rotation_xyzw=estimated_rotation.as_quat().tolist(),
        simulated_actual_grip_offset_m=actual_offset.tolist(),simulated_actual_grip_rotation_xyzw=actual_rotation.as_quat().tolist(),
        geometric_side_clearance_m=.02,final_release_offset_m=.005,
        collision='Original CAD/FCL every20ms; initial separation uses original suction/source exemption only, strict empty route after verified clearance',
        scope='Ideal rigid suction and frozen actual release; no contact dynamics or hardware',
        committed_prefix=len(occupied),motion_callback_replanning=False)
    report.setdefault('placements',[]).append(item)
    shared_events = report.setdefault('events',[])
    plans = []
    options = getattr(r, 'planner_options', {})
    counters = dict(ik_calls=0, endpoint_ik_calls=0, endpoint_unresolved=0,
        segment_cache_hits=0, segment_cache_misses=0, cad_calls=0,
        endpoint_filter_s=0., ik_s=0., cad_s=0., interpolation_s=0.)
    item['optimizer'] = counters
    item['planning_method'] = 'Sparse contact anchors, endpoint-first LM, exact-context segment cache, coarse IK knots + dense joint interpolation/CAD'
    segment_cache = {}
    publish()

    def failed(context, error):
        item['failures'].append(dict(**context,**error))
        item['planning_progress']=dict(**context,last_failure_stage=error.get('stage'),failures=len(item['failures']))
        # Publish only the physical current state, never a hypothetical release.
        restore_planning_context()
        publish()
        print(f"{label} placement search: candidate={context.get('rank')} phase={error.get('stage')} reason={error.get('reason')} pairs={error.get('pairs',[])}",flush=True)

    def cad(q):
        tick = time.perf_counter();counters['cad_calls'] += 1
        valid = r.validate(q)
        counters['cad_s'] += time.perf_counter()-tick
        if valid:
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
        # Cache is local to this scene/prefix; empty paths also bind release pose
        # and the detachment exemption, so a loaded result is never reused empty.
        context = (r.w.held, r.collision.current_source,
                   r.state.body_q.numpy()[held].tobytes() if r.w.held is None else b'loaded')
        key = (np.asarray(qstart).tobytes(), np.asarray(position).tobytes(),
               np.asarray(quaternion).tobytes(), context)
        if key in segment_cache:
            counters['segment_cache_hits'] += 1
            cached, error = segment_cache[key]
            if error:
                return None, dict(error, stage=name, cached=True)
            path, event = cached
            return (name, path, dict(event,stage=name,planning_wall_s=time.perf_counter()-start,cache_hit=True)), None
        counters['segment_cache_misses'] += 1
        r.ik.model.joint_q.assign(np.asarray(qstart))
        newton.eval_fk(r.ik.model,r.ik.model.joint_q,r.ik.model.joint_qd,r.ik.model)
        tcp = r.ik.model.body_q.numpy()[r.ik.contact_body].copy()
        position = np.array(position); quaternion = np.array(quaternion)
        duration = max(float(np.linalg.norm(position-tcp[:3]))/.30,
                       float((R.from_quat(tcp[3:]).inv()*R.from_quat(quaternion)).magnitude()),.02)
        distance = float(np.linalg.norm(position-tcp[:3]))
        angle = float((R.from_quat(tcp[3:]).inv()*R.from_quat(quaternion)).magnitude())
        knots = max(1,int(np.ceil(distance/options.get('knot_step_m',.08))),int(np.ceil(angle/.15)))
        samples = max(2,int(np.ceil(duration/.02)))
        rot = Slerp([0.,1.],R.from_quat([tcp[3:],quaternion])); q = np.array(qstart).copy(); controls=[q.copy()]
        for index, alpha in enumerate(np.linspace(0.,1.,knots+1)[1:]):
            target = tcp[:3]*(1-alpha)+position*alpha
            tick = time.perf_counter();counters['ik_calls'] += 1
            ok,q = r.ik._solve_once(target,rot(alpha).as_quat(),q,check_collision=False)
            counters['ik_s'] += time.perf_counter()-tick
            if not ok:
                error = dict(stage=name,reason='native_IK_unsolved',sample=index,
                    target_world_m=target.tolist(),diagnostics=dict(r.ik.last_diagnostics),q_rad=q.tolist())
                segment_cache[key]=(None,error)
                return None,error
            controls.append(q.copy())
        tick = time.perf_counter()
        path = PchipInterpolator(np.linspace(0.,1.,knots+1),np.asarray(controls),axis=0)(np.linspace(0.,1.,samples+1)[1:])
        counters['interpolation_s'] += time.perf_counter()-tick
        for index,q in enumerate(path):
            pairs = cad(q)
            if pairs:
                error = dict(stage=name,reason='original_CAD_rejected',sample=index,pairs=pairs,
                    contact_details=contact_details(),q_rad=q.tolist())
                segment_cache[key]=(None,error)
                return None,error
        event = dict(stage=name,planning_wall_s=time.perf_counter()-start,reference_s=len(path)*.02,frames=len(path),ik_knots=knots,cache_hit=False)
        segment_cache[key]=((path,event),None)
        return (name,path,event),None

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
        prior_class = SparseCompactPrior
        if options.get('layout_prior') == 'contact':
            from straight_place.contact_layout import ContactLayoutPrior
            prior_class = ContactLayoutPrior
        prior=prior_class(r.cfg,h,max_box_top_m=options.get('max_box_top_m',1.),ordering=options.get('candidate_order','greedy'),max_candidates=options.get('max_candidates',10),
                          neighbor_gap_m=options.get('neighbor_gap_m',0.),
                          min_support_coverage=options.get('min_support_coverage',.8),
                          **({'layer_preference_band_m':options.get('layer_preference_band_m',0.)} if options.get('layout_prior')=='contact' else {}),
                          use_endpoint_sphere=True)
        item['layout_prior'] = options.get('layout_prior', 'frontier')
        prior.horizontal_first=True;prior.row_first=True;prior.contact_first=False
        prior.horizontal_row_first=options.get('horizontal_row_first',False)
        prior.physical_padding_xy_m=options.get('physical_padding_xy_m',0.)
        # Task-level clearance, identical for all items; no actor-pose correction.
        prior.lateral_clearance_m=.02
        if options.get('record_decisions', False):
            from straight_place.candidate_trace import snapshot
            def record_candidates(data):
                trace = snapshot(prior, data)
                trace['generated_monotonic_s'] = time.perf_counter()
                trace['source'] = 'Actual attached-arm planning query, including original sphere and clearance'
                item['candidate_trace'] = trace
            prior.trace_sink = record_candidates
        candidates,stats=prior.candidates(dimensions,list(occupied))
        item['candidate_stats']=stats
        observation=np.array(taught_pose('observation'))
        taught_lift=np.array(taught_pose('post_pick_lift'))
        taught_entry=np.array(taught_pose('container_entry'))
        endpoint_batch = {}
        if options.get('endpoint_backend','scalar') == 'batch':
            from straight_place.batch_pose_ik import solve as batch_solve
            positions=[];rotations=[];seeds=[];identities=[]
            for rank,b in enumerate(candidates):
                yaw=-np.pi/2 if tuple(b.dims)==(dimensions[1],dimensions[0],dimensions[2]) else 0.
                for heading in (0.,np.pi):
                    tool_rotation=cr*R.from_euler('z',yaw+heading)*estimated_rotation.inv()
                    endpoint=np.asarray(b.pos)+np.asarray(b.dims)/2+origin+[0.,0.,.005]
                    position=translation+cr.apply(endpoint)-tool_rotation.apply(estimated_offset)
                    for seed in (base_q,q0):
                        positions.append(position);rotations.append(tool_rotation.as_quat());seeds.append(seed)
                        identities.append((rank,heading))
            tick=time.perf_counter()
            answers,batch_stats=batch_solve(r.ik,positions,rotations,seeds)
            counters['endpoint_filter_s'] += time.perf_counter()-tick
            counters['endpoint_ik_calls'] += len(positions)
            item['endpoint_batch_stats']=batch_stats
            for identity,answer in zip(identities,answers):
                previous=endpoint_batch.get(identity)
                if previous is None or (answer['ok'] and not previous['ok']):endpoint_batch[identity]=answer
        selected=None
        for rank,b in enumerate(candidates):
            item['planning_progress']=dict(rank=rank,candidates=len(candidates),failures=len(item['failures']))
            publish()
            yaw=-np.pi/2 if tuple(b.dims)==(dimensions[1],dimensions[0],dimensions[2]) else 0.
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
                if options.get('insert_then_shift', False):
                    entry_center = taught_entry[:3]+tool_rotation.apply(estimated_offset)
                    entry_corner = cr.inv().apply(entry_center-translation)-origin-np.array(b.dims)/2
                    entry_y = min(max(entry_corner[1], h.door_y0), h.door_y1-b.dims[1])
                    routes = []
                    for height in heights:
                        template = p.route_to(b, list(occupied), h, min_entry_z=height)
                        if template is None:
                            continue
                        transit_z = template[0][2]
                        route = [(template[0][0], entry_y, transit_z),
                                 (b.pos[0], entry_y, transit_z),
                                 (b.pos[0], b.pos[1], transit_z), tuple(b.pos)]
                        if route not in routes and p.path_clear(b.dims, route, list(occupied), h):
                            routes.append(route)
                # Reach sphere is only broadphase. A cheap pose solve precedes
                # any full route. LM failure is unresolved, not a proof of no IK.
                endpoint = np.asarray(b.pos).copy();endpoint[2] += .005
                ep_position,ep_rotation = target(endpoint)
                tick=time.perf_counter();endpoint_ok=False
                batch_answer=endpoint_batch.get((rank,heading))
                if batch_answer is not None:
                    endpoint_ok=batch_answer['ok'];ep_q=batch_answer['q']
                    ep_diagnostics=batch_answer['diagnostics']
                else:
                    for seed in (base_q,q0):
                        counters['endpoint_ik_calls'] += 1
                        endpoint_ok,ep_q = r.ik._solve_once(ep_position,ep_rotation,seed,check_collision=False)
                        if endpoint_ok:break
                    ep_diagnostics=dict(r.ik.last_diagnostics)
                    counters['endpoint_filter_s'] += time.perf_counter()-tick
                if not endpoint_ok:
                    counters['endpoint_unresolved'] += 1
                    failed(dict(rank=rank,heading_deg=float(np.degrees(heading)),route_index=None),
                        dict(stage=label+'_ENDPOINT_PREFILTER',reason='endpoint_IK_unresolved',diagnostics=ep_diagnostics))
                    continue
                restore_planning_context()
                ep_pairs = cad(ep_q)
                if ep_pairs:
                    failed(dict(rank=rank,heading_deg=float(np.degrees(heading)),route_index=None),
                        dict(stage=label+'_ENDPOINT_PREFILTER',reason='endpoint_CAD_rejected',pairs=ep_pairs))
                    continue
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
                        name = f'{label}_PLACE_LEG{leg}'
                        if options.get('insert_then_shift', False) and leg:
                            name = label+('_INSERT_DEPTH', '_SHIFT_LATERAL', '_LOWER_FINAL')[leg-1]
                        goals.append((name,position,quaternion))
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
                    if options.get('release_vertical_lift_m') is not None:
                        world_lift = np.array([0., 0., options['release_vertical_lift_m']])
                        variants = [('direct', cr.inv().apply(world_lift).tolist())]
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
            selected_monotonic_s=time.perf_counter(),
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
        extent=abs((cr.inv()*R.from_quat(release[3:])).as_matrix())@np.array(dimensions)
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
