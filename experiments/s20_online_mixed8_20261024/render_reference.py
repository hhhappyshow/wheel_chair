"""Native Newton rendering of saved model/FK traces.

This reads saved states only; no new planning, dynamics or robot execution.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
from compare import UPSTREAM
from container_demo import make_ground_grid
from online_packing_real import Hull
from online_packing_newton import make_model
import newton
import newton.viewer
import warp as wp
import yaml
import pyglet


def wall_clock_indices(wall_times_s,fps,state_count):
    times=np.asarray(wall_times_s,dtype=float)
    if fps<=0 or times.shape!=(state_count,) or not len(times) or not np.isfinite(times).all() or np.any(np.diff(times)<0):
        raise ValueError('wall_times_s must contain one finite, nondecreasing timestamp per recorded state; fps must be positive')
    times=times-times[0]
    # Avoid an extra output frame from sub-nanosecond subtraction roundoff.
    video_times=np.arange(int(np.ceil(times[-1]*fps-1e-9))+1)/fps
    return np.clip(np.searchsorted(times,video_times+1e-9,side='right')-1,0,state_count-1),float(times[-1])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--run-folder",type=Path)
    parser.add_argument("--camera-panel",action="store_true",help="Saved observation plus three native views")
    parser.add_argument("--box-colors",action="store_true",help="Display-only per-object colors, no geometry/state changes")
    parser.add_argument("--cell-aligned",action="store_true",help="Whole-cell overview and aligned top/side views")
    parser.add_argument("--single-view",action="store_true",help="One native perspective, not a three-view composite")
    parser.add_argument("--cutaway",action="store_true",help="Hide container skin for visual inspection; keep its CAD edges")
    parser.add_argument("--suitcase-assets",action="store_true",help="Reconstruct the authored suitcase RGB/depth mesh assets used by measured-pose pick")
    parser.add_argument('--site-urdf',type=Path,help='Use the same expanded site model as the recorded run')
    parser.add_argument('--cuboids',action='store_true',help='Display original box collision proxies instead of textured skins; no state changes')
    parser.add_argument("--stride",type=int,default=12)
    parser.add_argument("--fps",type=int,default=12)
    parser.add_argument("--wall-clock",action="store_true",help="Use recorded wall_times_s at FPS, holding states across planning waits; ignores stride")
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    bundle=UPSTREAM/"policy/demo_data/site_four_colors_right160_observe120"
    report=json.loads((args.run_folder/"result.json").read_text()) if args.run_folder else {}
    inputs=json.loads((bundle/"input.json").read_text()) if not args.run_folder else {"boxes":report.get("source_catalog_m",[[.55,.4,.25]]*3)}
    # Unrevealed slots stay hidden in the recorded states; keep model body order.
    inputs["boxes"]=list(inputs["boxes"])
    inputs["boxes"] += [[.55,.4,.25]] * max(0,report.get("asset_slots",len(inputs["boxes"]))-len(inputs["boxes"]))
    scene=args.run_folder/"scene_tf.yaml" if args.run_folder and (args.run_folder/"scene_tf.yaml").is_file() else bundle/"scene_tf.yaml"
    hull=Hull.from_config(yaml.safe_load(scene.read_text()))
    sample=dict(container=hull.as_dict(),steps=[dict(incoming=dict(id=f"B{i+1}",dims=d)) for i,d in enumerate(inputs["boxes"])])
    data=np.load((args.run_folder or bundle)/"trajectory.npz")
    wall_duration=None
    if args.wall_clock:
        if 'wall_times_s' not in data:raise ValueError('--wall-clock requires trajectory.npz wall_times_s; no motion-time fallback')
        indices,wall_duration=wall_clock_indices(data['wall_times_s'],args.fps,len(data['body_q']))
    else:
        indices=np.unique(np.r_[np.arange(0,len(data["body_q"]),args.stride),len(data["body_q"])-1])
    wp.set_device("cuda:0")
    if args.suitcase_assets:
        sys.path.insert(0,str(Path(__file__).resolve().parent/'perception'))
        from suitcase_capture import make_runtime
        model=make_runtime(args.output,inputs['boxes'],site_urdf=args.site_urdf).model
    else:
        model,_,_,_=make_model(sample,UPSTREAM/"policy/vendor/roboticarm",scene)
    if args.box_colors:
        colors=model.shape_color.numpy();bodies=model.shape_body.numpy();palette=[(.17,.65,.67),(.9,.59,.22),(.66,.4,.7),(.4,.7,.35),(.85,.35,.4),(.35,.55,.85),(.8,.7,.3),(.65,.65,.65)]
        for shape,body in enumerate(bodies):
            label=model.body_label[body] if body>=0 else ""
            if label.startswith("B") and label[1:].isdigit():
                colors[shape]=palette[(int(label[1:])-1)%len(palette)]
        model.shape_color.assign(colors)
    if args.cuboids:
        flags=model.shape_flags.numpy()
        for i,label in enumerate(model.shape_label):
            if label.endswith('_collision_box'):flags[i]|=int(newton.ShapeFlags.VISIBLE)
            elif label.endswith('_suitcase_visual'):flags[i]&=~int(newton.ShapeFlags.VISIBLE)
        model.shape_flags.assign(flags)
    if list(data["body_labels"])!=list(model.body_label):raise ValueError("Recorded model body order differs")
    state=model.state();viewer=newton.viewer.ViewerGL(width=1280 if args.single_view else 640,height=960 if args.single_view else 480,headless=True);viewer.set_model(model);viewer.show_ui=False
    import trimesh
    from scipy.spatial.transform import Rotation as R
    shell=model.shape_label.index("container_collision_full")
    visual=model.shape_label.index("container_visual")
    src=model.shape_source[shell]
    mesh=trimesh.Trimesh(np.asarray(src._vertices)*model.shape_scale.numpy()[shell],
                        np.asarray(src._indices).reshape(-1,3),process=True)
    edges=mesh.face_adjacency_edges[mesh.face_adjacency_angles>.2]
    xf=model.shape_transform.numpy()[shell]
    verts=R.from_quat(xf[3:]).apply(mesh.vertices)+xf[:3]
    starts=wp.array(verts[edges[:,0]].astype(np.float32),dtype=wp.vec3)
    ends=wp.array(verts[edges[:,1]].astype(np.float32),dtype=wp.vec3)
    visual_name=viewer._shape_to_batch[visual].name
    views=[((3.5,-4.,2.7),-12.,138.),((-1.,0.,6.),-89.,0.),((-1.,-4.,1.6),0.,90.)]
    if args.cell_aligned:
        target=np.array([-.9,.45,1.05]);positions=[np.array([3.3,3.7,3.8]),np.array([-.9,.45,6.]),np.array([-.9,-5.,1.5])]
        views=[]
        for k,pos in enumerate(positions):
            delta=target-pos
            if k==1:pitch=np.degrees(np.arctan2(delta[0],np.linalg.norm(delta[1:])));yaw=np.degrees(np.arctan2(delta[2],delta[1]))
            else:pitch=np.degrees(np.arctan2(delta[2],np.linalg.norm(delta[:2])));yaw=np.degrees(np.arctan2(delta[1],delta[0]))
            views.append((pos,pitch,yaw))
    name="online_newton" if args.run_folder else "classmate_reference"
    if args.single_view:views=views[:1]
    output=args.output/((name+"_perspective" if args.single_view else "observed_cycle_four_views" if args.camera_panel else name+"_three_views")+".mp4")
    size="1280x960" if args.single_view or args.camera_panel else "1920x480"
    enc=subprocess.Popen(["ffmpeg","-y","-loglevel","error","-f","rawvideo","-pix_fmt","rgb24","-s",size,"-r",str(args.fps),"-i","-","-c:v","libx264","-preset","fast","-crf","22","-pix_fmt","yuv420p","-movflags","+faststart",str(output)],stdin=subprocess.PIPE)
    previous_frame=None;combined=None;native_rendered_states=0
    for output_frame,frame in enumerate(indices):
        if frame!=previous_frame:
            combined=render_frame(viewer,state,data,frame,views,args,visual_name,starts,ends)
            previous_frame=frame;native_rendered_states+=1
        enc.stdin.write(combined.tobytes())
        if output_frame==len(indices)-1:
            from PIL import Image
            Image.fromarray(combined).save(args.output/(name+"_final.png"))
    enc.stdin.close();code=enc.wait();viewer.close()
    if code:raise RuntimeError("Encoder failed")
    (args.output/(name+"_render.json")).write_text(json.dumps(dict(source=str(args.run_folder or bundle),recorded_frames=len(data["body_q"]),rendered_frames=len(indices),native_rendered_states=native_rendered_states,fps=args.fps,stride=args.stride,motion_speedup=None if args.wall_clock else args.stride*.02*args.fps,wall_clock_playback=args.wall_clock,include_planning_waits=args.wall_clock,duration_basis='wall_times_s elapsed from first recorded frame; latest actual state held across waits' if args.wall_clock else 'recorded frame stride at output FPS; planning waits excluded',source_wall_duration_s=wall_duration,video_duration_s=len(indices)/args.fps,scope=__doc__,display_only_object_colors=args.box_colors,orthographic_views="CAD feature-edge shell, visual occluder hidden; collision geometry unmodified",new_physical_validation=False),indent=2))
    print(output)


def render_frame(viewer,state,data,frame,views,args,visual_name,starts,ends):
    state.body_q.assign(data["body_q"][frame]);state.joint_q.assign(data["joint_q"][frame]);panels=[]
    for view_index,(pos,pitch,yaw) in enumerate(views):
        viewer.camera.up_axis=0 if args.cell_aligned and view_index==1 else 2
        viewer.set_camera(pos=wp.vec3(*pos),pitch=pitch,yaw=yaw)
        for _ in range(2):
            viewer.begin_frame(frame/30);viewer.log_state(state)
            cutaway=view_index>0 or args.cutaway
            viewer.objects[visual_name].hidden=cutaway
            viewer.log_lines("cad_shell",starts if cutaway else None,
                             ends if cutaway else None,(.45,.65,.75) if cutaway else None)
            viewer.end_frame()
        panel=viewer.get_frame().numpy()
        panels.append(np.rot90(panel,2) if args.cell_aligned and view_index==1 else panel)
    if args.camera_panel:
        from PIL import Image,ImageOps
        observation="camera_rgb_0.png" if str(data["stage"][frame]) in ("OBSERVE","ADJUST_VIEW") else "camera_rgb.png"
        camera=ImageOps.pad(Image.open(args.run_folder/observation).convert("RGB"),(640,480),color=(34,41,49))
        combined=np.concatenate([np.concatenate([np.array(camera),panels[0]],axis=1),np.concatenate(panels[1:],axis=1)],axis=0)
    else:combined=np.concatenate(panels,axis=1)
    return combined


if __name__=="__main__":main()
