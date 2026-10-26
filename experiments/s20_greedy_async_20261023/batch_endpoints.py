"""Official Newton batched IK, with the existing tolerances and two seeds."""
import time
import numpy as np
import warp as wp
from newton import ik as nik
from newton_ik_planner import IK_ROTATION_TOLERANCE


def solve(ik,rows,source_dims,hull,domain,yaw_offsets=(0.,)):
    start=time.perf_counter();positions=[];rotations=[];seeds=[]
    for box in rows:
        yaw=ik.box_yaw(source_dims,box.dims)
        target,wyaw=ik.local_contact_world(box.pos,box.dims,yaw,hull)
        node=min(domain,key=lambda n:np.linalg.norm(np.array(n["contact_world_m"])-target)+abs(n["box_yaw_rad"]-yaw))
        for offset in yaw_offsets:
            for seed in (node["joint_q"],ik.q_default):
                seed=np.array(seed,dtype=np.float32).copy()
                if offset:
                    seed[5]=(seed[5]+offset+np.pi)%(2*np.pi)-np.pi
                positions.append(target);rotations.append(ik.tool_orientation(wyaw+offset));seeds.append(seed)
    if not rows:return [],dict(wall_s=0.,problems=0)
    positions=np.asarray(positions);rotations=np.asarray(rotations)
    objectives=[nik.IKObjectivePosition(link_index=ik.contact_body,link_offset=wp.vec3(0,0,0),
                    target_positions=wp.array(positions,dtype=wp.vec3),weight=1.),
                nik.IKObjectiveRotation(link_index=ik.contact_body,link_offset_rotation=wp.quat_identity(),
                    target_rotations=wp.array(rotations,dtype=wp.vec4),weight=.7),
                nik.IKObjectiveJointLimit(joint_limit_lower=ik.model.joint_limit_lower,
                    joint_limit_upper=ik.model.joint_limit_upper,weight=10.)]
    solver=nik.IKSolver(model=ik.model,n_problems=len(seeds),objectives=objectives,
                        lambda_initial=.1,jacobian_mode=nik.IKJacobianType.ANALYTIC)
    q=wp.array(np.array(seeds,dtype=np.float32),dtype=wp.float32)
    solver.step(q,q,iterations=35)
    solutions=q.numpy();poses=solver.body_q.numpy()[:,ik.contact_body]
    pe=np.linalg.norm(poses[:,:3]-positions,axis=1)
    re=2*np.arccos(np.clip(abs(np.sum(poses[:,3:]*rotations,axis=1)),0,1))
    lower=ik.model.joint_limit_lower.numpy()[:6];upper=ik.model.joint_limit_upper.numpy()[:6]
    limits=((solutions[:,:6]>=lower-1e-4)&(solutions[:,:6]<=upper+1e-4)).all(axis=1)
    good=(pe<=ik.position_tolerance_m)&(re<=IK_ROTATION_TOLERANCE)&limits
    answers=[]
    for i in range(len(rows)):
        variants=[]
        for j,offset in enumerate(yaw_offsets):
            first=2*(i*len(yaw_offsets)+j);pair=[first,first+1];viable=[k for k in pair if good[k]]
            chosen=viable[0] if viable else min(pair,key=lambda k:pe[k]+.7*re[k])
            variants.append(dict(ok=bool(viable),q=solutions[chosen].copy(),yaw_offset_rad=float(offset),
                                diagnostics=dict(position_error_m=float(pe[chosen]),rotation_error_rad=float(re[chosen]),
                                                 joint_limits=bool(limits[chosen]),collision_free=None)))
        answer=next((v for v in variants if v["ok"]),variants[0]).copy()
        answer["variants"]=variants;answers.append(answer)
    return answers,dict(wall_s=time.perf_counter()-start,problems=len(seeds),candidates=len(rows),
                        converged=sum(a["ok"] for a in answers),iterations=35,seeds_per_orientation=2,orientations=len(yaw_offsets),
                        scope="batched endpoint IK only; collision and full loaded/empty routes checked separately; failed seeds are not an impossibility proof")
