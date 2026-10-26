"""Export same-cell CAD and conservative link-local sphere covers for cuRoboV2.

Sphere covers are only an experimental world-collision prefilter. Full CAD
checks remain the acceptance backend; neither this nor V2 replaces self checks.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml
from compare import UPSTREAM,SCENE,p
from online_packing_newton import make_model
import newton
import warp as wp


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-folder",type=Path,required=True)
    parser.add_argument("--full-cycle",action="store_true")
    args=parser.parse_args();wp.set_device("cuda:0")
    hull=p.Hull.from_config(yaml.safe_load(SCENE.read_text()))
    sample=dict(container=hull.as_dict(),steps=[dict(incoming=dict(id=f"B{i+1}",dims=[.55,.4,.25])) for i in range(3)])
    model,ids,_,_=make_model(sample,UPSTREAM/"policy/vendor/roboticarm",SCENE)
    data=np.load(args.run_folder/"trajectory.npz");boxes=list(ids.values());arm_count=min(boxes)
    scene=[];covers=[];bodies=[];labels=[];pitch=.05
    shape_body=model.shape_body.numpy();local=model.shape_transform.numpy();scale=model.shape_scale.numpy()
    for i in range(model.shape_count):
        if not model.shape_flags.numpy()[i]&2:continue
        body=int(shape_body[i]);name=model.shape_label[i];mesh=model.shape_source[i]
        if body>=0 and model.body_label[body].endswith("/payload_box"):continue
        xf=local[i];rot=R.from_quat(xf[3:])
        if mesh is not None and hasattr(mesh,"_vertices"):
            vertices=np.array(mesh._vertices)*scale[i];faces=np.array(mesh._indices).reshape(-1,3)
        elif int(model.shape_type.numpy()[i])==int(newton.GeoType.BOX):
            import trimesh
            m=trimesh.creation.box(extents=2*scale[i]);vertices,faces=m.vertices,m.faces
        else:continue
        if body<0:
            vertices=rot.apply(vertices)+xf[:3]
            scene.append(dict(name=name,vertices=vertices.tolist(),faces=faces.ravel().tolist(),pose=[0,0,0,1,0,0,0]))
            continue
        if body>=arm_count:continue
        if model.body_label[body].endswith(("/elfin_base","/elfin_base_link")):continue
        # Circumscribed spheres cover a uniform partition of each collider's
        # AABB. This bounds the mesh, but includes empty AABB regions and is
        # intentionally NOT claimed equivalent to triangle collision.
        lo,hi=vertices.min(axis=0),vertices.max(axis=0)
        counts=np.maximum(1,np.ceil((hi-lo)/pitch).astype(int));step=(hi-lo)/counts
        coords=[lo[k]+(np.arange(counts[k])+.5)*step[k] for k in range(3)]
        xyz=np.array(np.meshgrid(*coords,indexing="ij")).reshape(3,-1).T
        xyz=rot.apply(xyz)+xf[:3];radius=float(np.linalg.norm(step)/2)
        covers.extend(np.c_[xyz,np.full(len(xyz),radius)]);bodies.extend([body]*len(xyz));labels.extend([name]*len(xyz))
    # The carried box also receives a volume cover, with the exact supplied
    # dimensions; this does not rewrite its CAD footprint or inflate the task.
    counts=np.ceil(np.array([.55,.4,.25])/pitch).astype(int);step=np.array([.55,.4,.25])/counts
    coords=[-.5*d+(np.arange(c)+.5)*s for d,c,s in zip([.55,.4,.25],counts,step)]
    payload=np.array(np.meshgrid(*coords,indexing="ij")).reshape(3,-1).T
    radius=np.linalg.norm(step)/2
    local_spheres=np.array(covers);bodies=np.array(bodies)
    batches=[];metadata=[]
    for arrival in range(3):
        indices=np.flatnonzero(np.isin(data["stage"],["LIFT","RAISE_FOR_TRANSFER","CARRY_TO_ENTRANCE","PLACE"])) if args.full_cycle and arrival==0 else np.flatnonzero(data["stage"]==f"B{arrival+1}_insert")
        if not len(indices):continue
        poses=data["body_q"][indices]
        q=poses[:,bodies,3:];v=np.broadcast_to(local_spheres[:,:3],q.shape[:2]+(3,))
        xyz=poses[:,bodies,:3]+v+2*np.cross(q[:,:,:3],np.cross(q[:,:,:3],v)+q[:,:,3:]*v)
        cargo=poses[:,boxes[arrival]];r=R.from_quat(cargo[:,3:]);cargo_xyz=np.stack([rr.apply(payload)+pp[:3] for rr,pp in zip(r,cargo)])
        spheres=np.concatenate([np.concatenate([xyz,np.broadcast_to(local_spheres[:,3:4],(*xyz.shape[:2],1))],axis=2),
                                np.concatenate([cargo_xyz,np.full((*cargo_xyz.shape[:2],1),radius)],axis=2)],axis=1)
        batches.append(spheres.astype(np.float32))
        obstacles=[]
        for previous in range(arrival):
            pose=poses[0,boxes[previous]]
            obstacles.append(dict(name=f"B{previous+1}",dims=[.55,.4,.25],pose=[*pose[:3],pose[6],*pose[3:6]]))
        metadata.append(dict(arrival=arrival+1,previous_boxes=obstacles,indices=indices.tolist()))
    np.savez_compressed(args.run_folder/"gpu_spheres.npz",**{f"B{i+1}":b for i,b in enumerate(batches)})
    output=dict(scope=__doc__,scene_mesh=scene,batches=metadata,spheres_per_sample=int(batches[0].shape[1]),
                pitch_m=pitch,arm_shape_labels=sorted(set(labels)),shape_body_labels={model.shape_label[i]:model.body_label[b] for i,b in enumerate(shape_body) if b>=0},
                self_collision_included=False,coverage="bounding AABB volume sphere cover, conservative and potentially over-rejecting")
    (args.run_folder/"gpu_scene.json").write_text(json.dumps(output,default=lambda value:value.item()))
    print(json.dumps(dict(spheres=output["spheres_per_sample"],batches=len(batches),scene_meshes=len(scene))))


if __name__=="__main__":main()
