"""Native single-perspective CAD layout render, no robot execution claim."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml
from PIL import Image
from compare import UPSTREAM,p
from geometry_preview8 import PALETTE
from online_packing_newton import make_model
import newton.viewer
import warp as wp
import trimesh


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument("--folder",type=Path,required=True);args=ap.parse_args();out=args.folder
    data=json.loads((out/"result.json").read_text());cfg=yaml.safe_load((out/"scene_tf.yaml").read_text());h=p.Hull.from_dict(data["hull"])
    sample=dict(container=h.as_dict(),steps=[dict(incoming=dict(id=f"B{i+1}",dims=d,color_rgb=PALETTE[i%8])) for i,d in enumerate(data["source_catalog_m"])])
    wp.set_device("cuda:0");model,ids,tf,_=make_model(sample,UPSTREAM/"policy/vendor/roboticarm",out/"scene_tf.yaml",show_robot=False)
    flags=model.shape_flags.numpy();sb=model.shape_body.numpy()
    for i,body in enumerate(sb):
        if body>=0 and body not in ids.values() or model.shape_label[i] in ("loading_zone_marking","robot_pedestal"):
            flags[i]&=~1
    model.shape_flags.assign(flags);state=model.state();initial=state.body_q.numpy().copy()
    for body in ids.values():initial[body]=[100,100,-100,0,0,0,1]
    cr=R.from_euler("xyz",tf["rotation_rpy"]);translation=np.array(tf["translation"]);origin=np.array([h.door_x,h.inner_y0,h.floor_z])
    def world(point):return translation+cr.apply(origin+np.array(point))
    viewer=newton.viewer.ViewerGL(width=1500,height=1000,headless=True);viewer.set_model(model);viewer.show_ui=False
    shell=model.shape_label.index("container_collision_full");visual=model.shape_label.index("container_visual")
    src=model.shape_source[shell];mesh=trimesh.Trimesh(np.asarray(src._vertices)*model.shape_scale.numpy()[shell],np.asarray(src._indices).reshape(-1,3),process=True)
    edges=mesh.face_adjacency_edges[mesh.face_adjacency_angles>.2];xf=model.shape_transform.numpy()[shell]
    vertices=R.from_quat(xf[3:]).apply(mesh.vertices)+xf[:3]
    starts=wp.array(vertices[edges[:,0]].astype(np.float32),dtype=wp.vec3);ends=wp.array(vertices[edges[:,1]].astype(np.float32),dtype=wp.vec3)
    target=world([.6,.78,.43]);pos=world([-2.8,-2.0,2.5]);delta=target-pos
    viewer.set_camera(pos=wp.vec3(*pos),pitch=np.degrees(np.arctan2(delta[2],np.linalg.norm(delta[:2]))),yaw=np.degrees(np.arctan2(delta[1],delta[0])))
    poses=initial.copy()
    for step in data["steps"]:
        if "box" not in step:continue
        b=step["box"];yaw=-np.pi/2 if b["dims"][0]<b["dims"][1] else 0.
        poses[ids[b["id"]]]=np.r_[world(np.array(b["pos"])+np.array(b["dims"])/2),(cr*R.from_euler("z",yaw)).as_quat()]
        state.body_q.assign(poses)
        for _ in range(3):
            viewer.begin_frame(0);viewer.log_state(state)
            viewer.objects[viewer._shape_to_batch[visual].name].hidden=True
            viewer.log_lines("cad_shell",starts,ends,(.47,.64,.72));viewer.end_frame()
        Image.fromarray(viewer.get_frame().numpy()).save(out/f"layout3d_{b['id']}.png")
    Image.fromarray(viewer.get_frame().numpy()).save(out/"layout3d_final.png");viewer.close()
    (out/"layout3d_render.json").write_text(json.dumps(dict(source=str(out/"result.json"),native_newton_CAD=True,view="single perspective; container skin hidden to expose contents, original CAD edges shown",scope=__doc__,arm_execution=False),indent=2))
    print(out/"layout3d_final.png")


if __name__=="__main__":main()
