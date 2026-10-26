"""Fresh online packing -> Newton IK -> full CAD sampled loaded/empty motion.

Already-held initialization and frozen releases; not dynamics or physical Pick.
Old domain joint witnesses are seeds only. No SDK import or hardware commands.
"""
import argparse
import json
import shutil
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as R
import yaml

from compare import ROOT, UPSTREAM, SCENE, pool, old_score, contact_area, p, Box
from newton_ik_planner import NewtonIKWorkspace
from online_packing_newton import make_model
from roboticarm_newton import DEFAULT_SOURCE
from scene_frames import container_pose, rotation, transform_point
import newton
import warp as wp
sys.path.insert(0, str(ROOT/"experiments/s20_newton_fullcycle20_20261021"))
from collision import Collision


class TopWorkspace(NewtonIKWorkspace):
    def local_contact_world(self, lower_left, dims, yaw, hull):
        world, world_yaw = super().local_contact_world(lower_left, dims, yaw, hull)
        return world + rotation(container_pose(self.config)).apply([0.,0.,dims[2]/2]), world_yaw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--method", choices=("old_score", "greedy_first"), default="greedy_first")
    parser.add_argument("--ik-tolerance-m", type=float, default=.01)
    parser.add_argument("--seed-mode", choices=("warm", "cold"), default="warm")
    parser.add_argument("--duration-s", type=float, default=6.)
    parser.add_argument("--count", type=int, default=3)
    parser.add_argument("--endpoint-first", action="store_true")
    parser.add_argument("--release-contact", action="store_true",
                        help="Admit only intended final cargo surface contact, matching upstream1mm semantics")
    parser.add_argument("--prior-first", action="store_true",
                        help="Try matching finite domain witnesses first, retain all other candidates as fallback")
    parser.add_argument("--height-profile",choices=("depth","flat","pair","stack"),default="depth")
    parser.add_argument("--straight", action="store_true",
                        help="Aligned X insertion then Z descent; Cartesian continuation every 2cm")
    parser.add_argument("--direct-empty-return", action="store_true")
    parser.add_argument("--request",type=Path,help="Observed current dimensions; no hidden future arrivals")
    parser.add_argument("--regions",action="store_true")
    parser.add_argument("--compact-prior",action="store_true")
    parser.add_argument("--mixed20-24",action="store_true")
    parser.add_argument("--perceive-arrivals",action="store_true")
    parser.add_argument("--horizontal-first",action="store_true")
    parser.add_argument("--sensors-center",action="store_true")
    parser.add_argument("--batch-endpoints",action="store_true")
    parser.add_argument("--no-endpoint-hints",action="store_true",help="Historical q can seed IK, but no historical XYZ candidate is inserted")
    parser.add_argument("--placement-only",action="store_true",help="Return a loaded reference, NOT a placed/returned cycle")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    for name in ("newton_compare.py","region_search.py","compare.py","batch_endpoints.py","compact_prior.py","mixed_perception.py","observed_cycle.py"):
        shutil.copyfile(Path(__file__).with_name(name),args.output/("source_"+name))
    started = time.perf_counter()
    cfg = yaml.safe_load(SCENE.read_text())
    shutil.copyfile(SCENE,args.output/"scene_tf.yaml")
    hull = p.Hull.from_config(cfg)
    from compact_prior import CompactPrior
    compact=CompactPrior(cfg,hull)
    compact.horizontal_first=args.horizontal_first
    ct = container_pose(cfg)
    cr = rotation(ct)
    source = UPSTREAM/"policy/vendor/roboticarm"
    trace = (args.output/"ik.jsonl").open("w")
    ik = TopWorkspace(source, SCENE, device="cuda:0")
    ik.position_tolerance_m = args.ik_tolerance_m
    request=json.loads(args.request.read_text()) if args.request else {}
    dims = tuple(request.get("dimensions_m",(.55,.4,.25)))
    labels=([20,24]*((args.count+1)//2))[:args.count] if args.mixed20_24 else [20]*args.count
    catalog=[(.55,.4,.25) if label==20 else (.65,.45,.30) for label in labels] if args.mixed20_24 else [dims]*args.count
    manifest = dict(container=hull.as_dict(), steps=[dict(incoming=dict(id=f"B{i+1}",dims=d)) for i,d in enumerate(catalog)])
    model, body_ids, _, _ = make_model(manifest, source, SCENE)
    state, fk = model.state(), model.state()
    box_ids = list(body_ids.values())
    tcp = model.body_label.index("S20/suction_contact_frame")
    w = SimpleNamespace(model=model, state=state, fk=fk, boxes=box_ids,
                        q=model.joint_q.numpy().copy(), qidx=np.arange(6),
                        arm_count=min(box_ids), tcp=tcp, held=None,
                        grip_offset=np.array([0.,0.,dims[2]/2]), grip_rot=R.from_euler("x",np.pi))
    collision = Collision(w)
    # The upstream URDF has a legacy placeholder payload centered at TCP.
    # The explicit attached box below is its sole geometric replacement.
    for shape in list(collision.objs):
        body = collision.bodies[shape]
        if body >= 0 and model.body_label[body].endswith("/payload_box"):
            del collision.objs[shape]
            collision.arm.discard(shape)
    domain = json.loads((UPSTREAM/"policy/demo_data/site_four_colors_right160_observe120/domain.json").read_text())["poses"]
    domain_xyz={yaw:np.array([n["contact_local_m"] for n in domain if abs(n["box_yaw_rad"]-yaw)<1e-5])
                for yaw in (0.,-np.pi/2)}
    q_entry = np.asarray(domain[0]["joint_q"],dtype=np.float32)
    histories, qs, stages = [], [], []
    placed, placed_yaws, events, cycles = [], [], [], []
    times = dict(initialization_s=time.perf_counter()-started)
    q = q_entry.copy()
    bodies = state.body_q.numpy()
    for i in box_ids:
        bodies[i] = [100.,100.,100.,0.,0.,0.,1.]
    state.body_q.assign(bodies)

    def frame(qi, stage):
        w.q[:6] = qi
        state.joint_q.assign(w.q)
        newton.eval_fk(model,state.joint_q,state.joint_qd,state)
        bodies = state.body_q.numpy()
        for index,b in enumerate(placed):
            center = np.array(b.pos)+np.array(b.dims)/2+[hull.door_x,hull.inner_y0,hull.floor_z]
            yaw = placed_yaws[index]
            bodies[box_ids[index]] = np.r_[transform_point(ct,center),(cr*R.from_euler("z",yaw)).as_quat()]
        if w.held is not None:
            rt = R.from_quat(bodies[tcp,3:])
            bodies[w.held] = np.r_[bodies[tcp,:3]+rt.apply(w.grip_offset),(rt*w.grip_rot).as_quat()]
        state.body_q.assign(bodies)
        histories.append(bodies.copy());qs.append(np.asarray(qi).copy());stages.append(stage)

    def ranked_candidates():
        if args.compact_prior:
            rows,stats=compact.candidates(dims,placed,args.height_profile)
            events.append(dict(stage="compact_region_prior",**stats))
            return rows
        if args.regions:
            from region_search import region_candidates
            rows,stats=region_candidates(dims,placed,hull,() if args.no_endpoint_hints else domain,profile=args.height_profile)
            events.append(dict(stage="region_search",**stats))
            return rows
        rows = pool(dims,placed,hull)
        # Finite reachable priors add interior seeds, never replace online
        # occupied-volume or support checks and never certify a new scene.
        for d in (dims,(dims[1],dims[0],dims[2])):
            for node in domain:
                x,y,_ = node["contact_local_m"]
                for z in sorted({0.}|{b.pos[2]+b.dims[2] for b in placed}):
                    b=Box("current",d,(round(x-d[0]/2,3),round(y-d[1]/2,3),z))
                    if hull.fits(b) and not any(p.collides(b,t) for t in placed) and (hull.floor_supports(b) or p.fully_supported(b,placed)):
                        rows.append(b)
        rows = list({(b.pos,b.dims):b for b in rows}.values())
        if args.method=="old_score":rows.sort(key=lambda b:old_score(b,placed,hull,"flat"))
        else:
            def score_key(b):
                depth=round(b.pos[0]+b.dims[0],3);layer=b.pos[2]/dims[2]
                tail=(-contact_area(b,placed,hull),b.pos[1])
                if args.height_profile=="flat":return (layer,-depth,*tail)
                if args.height_profile=="stack":return (-layer,-depth,*tail)
                if args.height_profile=="pair":return ((layer-1)**2+.1*layer,-depth,*tail)
                return (-depth,layer,*tail)
            rows.sort(key=score_key)
        if args.prior_first:
            # Stable partition, not a reachability exclusion or new motion
            # gate. Every hint still goes through actual IK + full CAD checks.
            def near_witness(b):
                top=np.array(b.pos)+np.array(b.dims)/2;top[2]+=b.dims[2]/2
                yaw=ik.box_yaw(dims,b.dims)
                distance=np.linalg.norm(domain_xyz[yaw]-top,axis=1).min()
                return distance>args.ik_tolerance_m+1e-8
            def hint_key(b):
                layer=b.pos[2]/dims[2]
                if args.height_profile=="flat":return (layer,near_witness(b))
                if args.height_profile=="stack":return (-layer,near_witness(b))
                if args.height_profile=="pair":return ((layer-1)**2+.1*layer,near_witness(b))
                return (near_witness(b),)
            rows.sort(key=hint_key)
        return rows

    def cad_error(qi,allow_contact=False):
        filters=collision.filters.copy()
        try:
            while not collision.validate(qi):
                import fcl
                depths=[];cargo_contact=True;hits=list(collision.last_hits)
                for pair in hits:
                    ids=[model.shape_label.index(name) for name in pair]
                    result=fcl.CollisionResult()
                    fcl.collide(collision.objs[ids[0]],collision.objs[ids[1]],
                                fcl.CollisionRequest(num_max_contacts=8,enable_contact=True),result)
                    depths.append(max((c.penetration_depth for c in result.contacts),default=float("inf")))
                    cargo_contact &= all(name.startswith("B") and name.endswith("_collision_box") for name in pair)
                if not(allow_contact and cargo_contact and max(depths,default=1.)<=.001):
                    return dict(reason="sampled full CAD collision",hits=hits,depth_m=depths)
                for pair in hits:
                    ids=[model.shape_label.index(name) for name in pair]
                    collision.filters.add(tuple(sorted(ids)))
            return None
        finally:collision.filters=filters

    def solve_route(box, route, start, fixed_start=False, duration_s=None,yaw_offset=0.):
        duration=args.duration_s if duration_s is None else duration_s
        descent_start=len(route)-1
        while descent_start>0 and np.linalg.norm(np.array(route[descent_start-1])[:2]-np.array(route[-1])[:2])<1e-7:
            descent_start-=1
        intended_descent=(args.compact_prior and not fixed_start and route[-1][2]<route[descent_start][2])
        path=[]; qi=start.copy()
        yaw=ik.box_yaw(dims,box.dims)
        for point_index,point in enumerate(route):
            target, world_yaw=ik.local_contact_world(point,box.dims,yaw,hull)
            world_yaw+=yaw_offset
            if not path and fixed_start:
                path.append(qi.copy())
                continue
            # Use the nearest current-frame witness only as an IK seed.
            if not path and args.seed_mode=="warm":
                local=np.array(point)+np.array(box.dims)/2;local[2]+=box.dims[2]/2
                node=min(domain,key=lambda n:np.linalg.norm(np.array(n["contact_local_m"])-local)+(abs(n["box_yaw_rad"]-yaw)))
                ik.q_last=np.array(node["joint_q"],dtype=np.float32)
                if yaw_offset:ik.q_last[5]=(ik.q_last[5]+yaw_offset+np.pi)%(2*np.pi)-np.pi
            else:ik.q_last=qi.copy()
            # Continue the reached branch; the author's pose-only cache does
            # not encode its seed. Exact CAD below replaces the approximate
            # static Newton contact call, not the collision requirement.
            ok, qi=ik._solve_once(target,ik.tool_orientation(world_yaw),ik.q_last,check_collision=False)
            if ok:ik.q_last=qi.copy()
            trace.write(json.dumps(dict(target_world=target.tolist(),ok=ok,diagnostics=ik.last_diagnostics))+"\n")
            if not ok:return None,dict(reason="IK/static collision",diagnostics=ik.last_diagnostics)
            error=cad_error(qi,args.release_contact and (point_index==len(route)-1 or intended_descent and point_index>=descent_start))
            if error:return None,error
            path.append(qi.copy())
        # Cartesian straight mode allocates time by Cartesian distance. A
        # corner is a deliberate segment boundary, never a stop at every IK knot.
        knots=np.array(path)
        length=np.r_[0.,np.cumsum(np.maximum(
            np.linalg.norm(np.diff(np.array(route),axis=0),axis=1) if args.straight
            else np.max(abs(np.diff(knots,axis=0)),axis=1),1e-5))]
        ts=length/length[-1]*duration
        spline=CubicSpline(ts,knots,bc_type=((1,np.zeros(6)),(1,np.zeros(6))))
        grid=spline(np.linspace(0,duration,max(2,int(np.ceil(duration/.02)))+1))
        # Refine with the same source geometry, not endpoint-only acceptance.
        for sample_index,qi in enumerate(grid):
            elapsed=duration*sample_index/(len(grid)-1)
            error=cad_error(qi,args.release_contact and (sample_index==len(grid)-1 or intended_descent and elapsed>=ts[descent_start]))
            if error:return None,error
        return grid,None

    def sample_route(route):
        sampled=[route[0]]
        for a,b in zip(route,route[1:]):
            a,b=np.array(a),np.array(b)
            count=max(2,int(np.ceil(np.linalg.norm(b-a)/.02))+1) if args.straight else max(2,int(np.linalg.norm(b-a)/.05)+1)
            for t in np.linspace(0,1,count)[1:]:
                sampled.append((a*(1-t)+b*t).tolist())
        return sampled

    def aligned_route(b):
        x,y,z=b.pos;lx,ly,lz=b.dims
        levels=sorted({max(z+.03,hull.door_z0,.4)}|
                      {max(z+.03,t.pos[2]+t.dims[2]+.03,.4) for t in placed})
        for height in levels:
            route=[(-lx-.10,y,height),(x,y,height),(x,y,z)]
            if p.path_clear(b.dims,route,placed,hull):return route
        return None

    report = dict(scope=__doc__,scene=str(SCENE.relative_to(ROOT)),method=args.method,
                  ik_tolerance_m=args.ik_tolerance_m,seed_mode=args.seed_mode,
                  endpoint_first=args.endpoint_first,
                  prior_first=args.prior_first,
                  batch_endpoints=args.batch_endpoints,
                  compact_prior=args.compact_prior,
                  source_catalog_m=catalog,source_size_labels=labels,horizontal_first=args.horizontal_first,
                  sensors_center=args.sensors_center,perceive_arrivals=args.perceive_arrivals,
                  historical_xyz_hints=not args.no_endpoint_hints,
                  height_profile=args.height_profile,
                  straight=args.straight,direct_empty_return=args.direct_empty_return,
                  placement_only=args.placement_only,
                  intended_final_cargo_contact_m=.001 if args.release_contact else 0.,
                  reference_duration_per_leg_s=args.duration_s,servo_grid_s=.02,
                  releases="frozen ideal box, not natural settling",
                  validation="Newton IK/static; full CAD FCL arm+self+payload per20ms; not continuous SVSDF proof",
                  events=events,cycles=cycles,timing=times)
    newton.eval_fk(model,state.joint_q,state.joint_qd,state)
    initial_poses=state.body_q.numpy();tcp_rot=R.from_quat(initial_poses[tcp,3:])
    sensor_ids=[i for i,name in enumerate(model.body_label) if name.endswith("/camera_link") or name.endswith("/mid360_mount_frame")]
    sensor_offset=tcp_rot.inv().apply(initial_poses[sensor_ids,:3].mean(axis=0)-initial_poses[tcp,:3])
    report["sensor_side_preference"]=dict(body_names=[model.body_label[i] for i in sensor_ids],offset_tcp_m=sensor_offset.tolist(),scope="soft centerline-facing preference between0/180deg equivalent tool headings; full CAD still decides feasibility")
    def inward_score(b,offset):
        yaw=ik.box_yaw(dims,b.dims);target,wyaw=ik.local_contact_world(b.pos,b.dims,yaw,hull)
        sensor_vector=R.from_quat(ik.tool_orientation(wyaw+offset)).apply(sensor_offset)
        toward=cr.apply([0,hull.flat_y/2-b.pos[1]-b.dims[1]/2,0])
        return float(sensor_vector@toward/(np.linalg.norm(sensor_vector)*np.linalg.norm(toward)+1e-12))
    perception=None
    if args.perceive_arrivals:
        from mixed_perception import MixedPerception
        perception_init=time.perf_counter()
        perception=MixedPerception(catalog,args.output/"perception")
        times["perception_initialization_s"]=time.perf_counter()-perception_init
    symmetry=(0.,np.pi) if args.sensors_center else (0.,)
    for arrival in range(args.count):
        dims=tuple(catalog[arrival]);vision=None
        if perception:
            try:vision=perception.observe(arrival,[-24,18,7,-32,29,-12,35,-6][arrival%8])
            except Exception as exc:
                cycles.append(dict(arrival=arrival+1,status="observation_failed",reason=str(exc)));break
            dims=tuple(float(v) for v in np.round(vision["observed"]["dimensions_m"],3))
            events.append(dict(stage="current_rgbd_model",arrival=arrival+1,wall_s=vision["wall_s"],dimensions_m=dims,yaw_world_deg=vision["observed"]["yaw_world_deg"]))
        w.grip_offset=np.array([0.,0.,catalog[arrival][2]/2])
        collision.current_source=arrival;collision.arrived=set(range(len(placed)));w.held=box_ids[arrival]
        query_start=time.perf_counter();rows=ranked_candidates();begin=time.perf_counter();selected=None;reasons={}
        candidate_s=begin-query_start
        endpoints=None
        if args.batch_endpoints:
            from batch_endpoints import solve as solve_endpoints
            endpoints,stats=solve_endpoints(ik,rows,dims,hull,domain,yaw_offsets=symmetry)
            events.append(dict(stage="batch_endpoints",arrival=arrival+1,**stats))
            from region_search import valid,greedy_key
            candidate_valid=(lambda b:compact.valid(b,placed)) if args.compact_prior else (lambda b:valid(b,placed,hull))
            candidate_key=(lambda b:compact.key(b,placed,args.height_profile)) if args.compact_prior else (lambda b:greedy_key(b,placed,hull,args.height_profile))
            # Refine around currently converged IK regions, not around rear
            # corners already unresolved by both seeds. Keep every fallback.
            for step in (.08,.04,.02):
                known={(b.pos,b.dims) for b in rows};extra=[]
                for b,a in [(b,a) for b,a in zip(rows,endpoints) if a["ok"]][:4]:
                    for dx,dy in ((step,0),(-step,0),(0,step),(0,-step)):
                        v=Box("current",b.dims,(round(b.pos[0]+dx,4),round(b.pos[1]+dy,4),b.pos[2]))
                        if (v.pos,v.dims) not in known and candidate_valid(v):
                            known.add((v.pos,v.dims));extra.append(v)
                answers,stats=solve_endpoints(ik,extra,dims,hull,domain,yaw_offsets=symmetry)
                events.append(dict(stage="batch_local_refinement",arrival=arrival+1,step_m=step,**stats))
                combined=sorted(zip(rows+extra,endpoints+answers),key=lambda pair:candidate_key(pair[0]))
                rows=[pair[0] for pair in combined];endpoints=[pair[1] for pair in combined]
        for k,b in enumerate(rows):
            if args.sensors_center:
                variants=sorted(endpoints[k]["variants"],key=lambda a:-inward_score(b,a["yaw_offset_rad"]))
                route=aligned_route(b) if args.straight else None
                if route is None:route=p.route_to(b,placed,hull,min_entry_z=.4)
                if route is None:continue
                for answer in variants:
                    if not answer["ok"]:continue
                    if cad_error(answer["q"],args.release_contact):continue
                    offset=answer["yaw_offset_rad"]
                    grid,error=solve_route(b,sample_route(route),q_entry,yaw_offset=offset)
                    if error:
                        reasons[error["reason"]]=reasons.get(error["reason"],0)+1
                        events.append(dict(arrival=arrival+1,candidate=k,yaw_offset_rad=offset,box=dict(pos=b.pos,dims=b.dims),error=error))
                        continue
                    selected=(b,grid,k,route,offset);break
                if selected:break
                continue
            if endpoints is not None:
                answer=endpoints[k]
                ok=answer["ok"]
                if ok:
                    ok=cad_error(answer["q"],args.release_contact) is None
                    answer["diagnostics"]["collision_free"]=ok
                if not ok:
                    reasons["batch endpoint unresolved/collision"]=reasons.get("batch endpoint unresolved/collision",0)+1
                    events.append(dict(arrival=arrival+1,candidate=k,box=dict(pos=b.pos,dims=b.dims),error=dict(reason="batch endpoint unresolved/collision",diagnostics=answer["diagnostics"])))
                    continue
                ik.q_last=answer["q"].copy()
            if args.endpoint_first and endpoints is None:
                yaw=ik.box_yaw(dims,b.dims)
                target,wyaw=ik.local_contact_world(b.pos,b.dims,yaw,hull)
                if args.seed_mode=="warm":
                    node=min(domain,key=lambda n:np.linalg.norm(np.array(n["contact_world_m"])-target)+abs(n["box_yaw_rad"]-yaw))
                    ik.q_last=np.array(node["joint_q"],dtype=np.float32)
                else:ik.q_last=ik.q_default.copy()
                ok,goal_q=ik.solve_world_pose(target,wyaw,check_collision=False)
                if ok:ok=cad_error(goal_q,args.release_contact) is None
                if not ok:
                    reasons["endpoint rejected"]=reasons.get("endpoint rejected",0)+1
                    events.append(dict(arrival=arrival+1,candidate=k,box=dict(pos=b.pos,dims=b.dims),error=dict(reason="endpoint rejected",diagnostics=ik.last_diagnostics)))
                    continue
            route=aligned_route(b) if args.straight else p.route_to(b,placed,hull,min_entry_z=.4)
            if route is None and args.compact_prior:
                route=p.route_to(b,placed,hull,min_entry_z=.4)
            if route is None:continue
            sampled=sample_route(route)
            grid,error=solve_route(b,sampled,q_entry)
            if error:
                reasons[error["reason"]]=reasons.get(error["reason"],0)+1
                events.append(dict(arrival=arrival+1,candidate=k,box=dict(pos=b.pos,dims=b.dims),error=error))
                continue
            selected=(b,grid,k,route);break
        planning=time.perf_counter()-begin
        if selected is None:
            cycles.append(dict(arrival=arrival+1,status="rejected",candidates=len(rows),planning_s=planning,candidate_s=candidate_s,query_total_s=planning+candidate_s,reasons=reasons));break
        b,grid,k,route=selected[:4];offset=selected[4] if len(selected)>4 else 0.
        if args.compact_prior:
            neighbor,wall=compact.contacts(b,placed)
            events.append(dict(stage="selected_compact_prior",arrival=arrival+1,neighbor_contact_m2=neighbor,wall_contact_m2=wall,rank_key=compact.key(b,placed,args.height_profile)))
        # This diagnostic initializes at the loaded entry configuration. It
        # does not claim to connect an arbitrary pickup or previous entry.
        for qi in grid:frame(qi,f"B{arrival+1}_insert")
        if args.placement_only:
            cycles.append(dict(arrival=arrival+1,status="validated_loaded_reference",box=dict(pos=b.pos,dims=b.dims),
                               planning_s=planning,candidate_s=candidate_s,query_total_s=planning+candidate_s,
                               candidates_tried=k+1,frames=len(grid),route=route,loaded_duration_s=args.duration_s))
            break
        placed.append(Box(f"B{arrival+1}",b.dims,b.pos));placed_yaws.append(ik.box_yaw(dims,b.dims));w.held=None
        collision.current_source=None;collision.arrived=set(range(len(placed)))
        if args.direct_empty_return:
            # Commit the same nominal frozen release before planning and
            # replaying the empty path, so both use one obstacle scene.
            frame(grid[-1],f"B{arrival+1}_release")
        retreat_start=time.perf_counter();retreat=grid[::-1];return_mode="reverse_loaded_reference";return_attempts=[]
        if args.direct_empty_return:
            ik.cache.clear()
            for empty_route in ([route[-1],route[0]],
                                [route[-1],(route[-1][0],route[-1][1],route[-1][2]+.10),route[0]]):
                length=lambda points:float(np.linalg.norm(np.diff(np.array(points),axis=0),axis=1).sum())
                empty_duration=args.duration_s*length(empty_route)/length(route)
                candidate,error=solve_route(b,sample_route(empty_route),grid[-1],fixed_start=True,duration_s=empty_duration,yaw_offset=offset)
                return_attempts.append(dict(route=empty_route,error=error))
                if candidate is not None:
                    retreat=candidate;return_mode="fresh_empty_cartesian";break
        retreat_ok=True
        for qi in retreat:
            # Fresh empty routes already passed the identical CAD checker.
            if return_mode!="fresh_empty_cartesian" and not collision.validate(qi):retreat_ok=False;break
            frame(qi,f"B{arrival+1}_empty_return")
        cycles.append(dict(arrival=arrival+1,status="kinematic_entry_place_return" if retreat_ok else "withdraw_failed",box=dict(pos=b.pos,dims=b.dims),planning_s=planning,candidate_s=candidate_s,query_total_s=planning+candidate_s,candidates_tried=k+1,frames=len(grid),route=route,return_mode=return_mode,return_attempts=return_attempts,return_query_s=time.perf_counter()-retreat_start,loaded_duration_s=args.duration_s,empty_duration_s=empty_duration if return_mode=="fresh_empty_cartesian" else args.duration_s,loaded_joint_travel_rad=float(np.abs(np.diff(grid,axis=0)).sum()),empty_joint_travel_rad=float(np.abs(np.diff(retreat,axis=0)).sum())))
        cycles[-1].update(size_label=labels[arrival],observed_dimensions_m=dims,layout_yaw_deg=float(np.degrees(ik.box_yaw(dims,b.dims)))%180,
                          tool_yaw_offset_deg=float(np.degrees(offset)),sensor_inward_score=inward_score(b,offset),
                          perception_s=vision["wall_s"] if vision else None,source_yaw_world_deg=vision["observed"]["yaw_world_deg"] if vision else None)
        if vision:cycles[-1]["raw_observed_dimensions_m"]=vision["observed"]["dimensions_m"]
        if not retreat_ok:break
        print(json.dumps(dict(stage="committed_kinematic_cycle",arrival=arrival+1,box=dict(pos=b.pos,dims=b.dims),planning_s=planning,candidates_tried=k+1)),flush=True)
        q_entry=grid[0].copy()
    report["wall_s"]=time.perf_counter()-started
    report["placed"]=sum(c["status"]=="kinematic_entry_place_return" for c in cycles)
    report["loaded_references"]=sum(c["status"]=="validated_loaded_reference" for c in cycles)
    (args.output/"result.json").write_text(json.dumps(report,indent=2))
    if histories:np.savez_compressed(args.output/"trajectory.npz",body_q=np.array(histories),joint_q=np.array(qs),stage=np.array(stages),body_labels=np.array(model.body_label))
    trace.close()
    print(json.dumps({k:report[k] for k in ("method","placed","wall_s","cycles")}),flush=True)


if __name__=="__main__":main()
