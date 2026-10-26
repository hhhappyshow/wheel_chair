"""Read-only measured-q FK and native empty-S20 scene portrait.

Default is CPU analysis. --render explicitly enables a native Newton/OpenGL
still; --orbit-frames N optionally adds a camera orbit around the static pose.
No hardware connection, planning, simulation stepping or calibration fitting.
"""
import argparse
import ast
import copy
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True
import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml


def tf(xyz=(0,0,0), rpy=(0,0,0)):
    out = np.eye(4)
    out[:3,:3] = R.from_euler('xyz',rpy).as_matrix()
    out[:3,3] = xyz
    return out


def origin(joint):
    e = joint.find('origin')
    return tf() if e is None else tf(np.fromstring(e.get('xyz','0 0 0'),sep=' '),np.fromstring(e.get('rpy','0 0 0'),sep=' '))


def original_empty_urdf(policy, vendor):
    # Execute only the original CPU URDF-construction functions. Importing
    # the full author module would import Newton/Warp before --render.
    tree = ast.parse((policy/'roboticarm_newton.py').read_text())
    names = {'xacro_properties','add_inertial','add_geometry','fixed_joint','make_robot_urdf'}
    namespace = dict(ET=ET,copy=copy,Path=Path,XACRO_NS='{http://www.ros.org/wiki/xacro}',
                     BOX_SIZE=(.35,.25,.18),BOX_MASS=5.)
    selected = ast.Module(body=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in names],type_ignores=[])
    exec(compile(selected,str(policy/'roboticarm_newton.py'),'exec'),namespace)
    source = vendor/'elfin_humble_ws/src'
    xml = ET.fromstring(namespace['make_robot_urdf'](source,source/'luggage_gazebo/models'))
    for e in list(xml):
        if e.tag == 'link' and e.get('name') == 'payload_box' or e.tag == 'joint' and e.get('name') == 'payload_grip_joint':
            xml.remove(e)
    return xml


def site_empty_urdf(path,upstream):
    """Keep current expanded site joints; resolve CAD locally, without ROS nodes."""
    xml=ET.parse(path).getroot()
    omitted_links={'world','elfin_base_link','elfin_dummy_gripper','payload_box'}
    omitted_joints={'world_base','elfin_base_joint','elfin_dummy_joint','payload_grip_joint'}
    for element in list(xml):
        if (element.tag not in ('link','joint','material') or
            element.tag=='link' and element.get('name') in omitted_links or
            element.tag=='joint' and element.get('name') in omitted_joints):
            xml.remove(element)
    for mesh in xml.iter('mesh'):
        uri=mesh.get('filename')
        if not uri.startswith('package://'):continue
        package,relative=uri.removeprefix('package://').split('/',1)
        candidates=[upstream/'src'/package/relative]
        if package=='luggage_description':
            suffix=Path(relative).relative_to('meshes')
            candidates.extend([upstream/'src/luggage_gazebo/models'/suffix,
                upstream/'src/luggage_gazebo/models/suction_panel/collision'/suffix.name])
        resolved=next((p for p in candidates if p.is_file()),None)
        if resolved is None:raise FileNotFoundError(uri)
        mesh.set('filename',str(resolved.resolve()))
    return xml


def forward(xml,q):
    frames = {'elfin_base':np.eye(4)}
    joints = xml.findall('joint')
    while True:
        added = 0
        for j in joints:
            parent,child = j.find('parent').get('link'),j.find('child').get('link')
            if parent not in frames or child in frames:
                continue
            joint = origin(j)
            if j.get('type') in ('revolute','continuous'):
                idx = int(j.get('name').removeprefix('elfin_joint'))-1
                axis = np.fromstring(j.find('axis').get('xyz'),sep=' ')
                motion = np.eye(4)
                motion[:3,:3] = R.from_rotvec(axis/np.linalg.norm(axis)*q[idx]).as_matrix()
                joint = joint@motion
            frames[child] = frames[parent]@joint
            added += 1
        if not added:
            break
    return frames


def packed(matrix):
    return dict(position_m=matrix[:3,3].tolist(),rpy_deg_xyz=R.from_matrix(matrix[:3,:3]).as_euler('xyz',degrees=True).tolist(),
                quaternion_xyzw=R.from_matrix(matrix[:3,:3]).as_quat().tolist(),T=matrix.tolist())


def controller_flange(sample,root):
    if 'flange_readback' in sample:
        f = sample['flange_readback']
        if 'T_base_flange' in f:
            return np.array(f['T_base_flange']), 'explicit T_base_flange'
        return tf(np.array(f['position_mm'])/1000,np.radians(f['rpy_deg'])), 'explicit mm / extrinsic xyz RPY degrees'
    if 'act_pos_raw' in sample:
        tcp = np.array(root.get('tcp_config_raw',[]),float)
        ucs = np.array(root.get('ucs_config_raw',[]),float)
        if tcp.shape != (6,) or ucs.shape != (6,) or np.any(tcp) or np.any(ucs):
            return None,'act_pos_raw not interpreted as flange: zero TCP/UCS not established'
        values = np.array(sample['act_pos_raw'][6:12],float)
        return tf(values[:3]/1000,np.radians(values[3:])), 'act_pos_raw[6:12], mm/xyz RPY degrees; controller TCP and UCS explicitly zero'
    return None,'no optional controller flange readback supplied'


def analyze(pose,xml,cfg,include_site_frames=False,driver_optical=None):
    sample = pose.get('samples',[pose])[-1]
    q = np.array(pose.get('q_rad',sample.get('q_rad',[])),float)
    if q.size == 0:
        q = np.radians(pose.get('q_deg',sample['q_deg']))
    if q.shape != (6,):
        raise ValueError('Expected six q_rad or q_deg, in elfin_joint1..6 order')
    fk = forward(xml,q)
    ped = cfg['pedestal']
    base = tf(np.array(ped['translation'])+[0,0,ped['size'][2]],
              [0,0,ped['rotation_rpy'][2]+cfg['robot']['rotation_rpy'][2]])
    c = next(t for t in cfg['static_transforms'] if t['child']=='container_link')
    uld = tf(c['translation'],c['rotation_rpy'])
    world = {'base':base,'flange':base@fk['elfin_end_link'],'tcp':base@fk['suction_contact_frame'],'uld':uld}
    if include_site_frames:
        world.update(world=np.eye(4),pedestal=tf(ped['translation'],ped['rotation_rpy']),
                     camera=base@fk['camera_link'])
        opening=next(t for t in cfg['static_transforms'] if t['child']=='container_opening_frame')
        world['opening']=uld@tf(opening['translation'],opening['rotation_rpy'])
        for short,name in (('optical','camera_color_optical_frame'),('lidar','mid360_mount_frame')):
            if name in fk:world[short]=base@fk[name]
        if driver_optical is not None:world['optical']=world['camera']@driver_optical
    comparisons = []
    for s in pose.get('samples',[pose]):
        actual,interpretation = controller_flange(s,pose)
        sq = np.array(s.get('q_rad',q),float) if 'q_deg' not in s else np.radians(s['q_deg'])
        flange = forward(xml,sq)['elfin_end_link']
        row = dict(interpretation=interpretation,remote_stamp_untrusted=s.get('stamp_utc'))
        if actual is not None:
            row.update(position_error_mm=float(np.linalg.norm(actual[:3,3]-flange[:3,3])*1000),
              orientation_error_deg=float(np.degrees((R.from_matrix(actual[:3,:3]).inv()*R.from_matrix(flange[:3,:3])).magnitude())),
              controller_base_flange=packed(actual))
        comparisons.append(row)
    frames = {name:dict(world=packed(w),base=packed(np.linalg.inv(base)@w),uld=packed(np.linalg.inv(uld)@w)) for name,w in world.items()}
    report = dict(q_rad=q.tolist(),q_deg=np.degrees(q).tolist(),
        captured_at_client_local=pose.get('captured_at_client_local'),
        timestamp_note='remote 1970 clock invalid; use supplied local capture timestamp only',
        frames=frames,controller_comparisons=comparisons,
        original_tool_mount=dict(T_flange_tcp=packed(np.linalg.inv(fk['elfin_end_link'])@fk['suction_contact_frame'])),
        empty_robot=True,cargo_count=0,payload_placeholder_removed=True,independent_calibration=False,
        assumptions='Current classmate pedestal/base/ULD TF and original tool CAD mounting; FK agreement does not validate world/ULD/tool calibration.',
        render_executed=False,hardware_connected=False,motion_sent=False)
    return q,world,report


def render(args,policy,vendor,empty_xml,q,world,report):
    import warp as wp
    import newton
    import newton.viewer
    wp.config.kernel_cache_dir = str(args.output/'warp_cache')
    wp.set_device('cuda:0')
    sys.path.insert(0,str(policy))
    import roboticarm_newton as native
    native.make_robot_urdf = lambda *a,**k: ET.tostring(empty_xml,encoding='unicode')
    builder = newton.ModelBuilder()
    native.add_cell_scene(builder,vendor,scene_config_path=args.scene)
    model = builder.finalize()
    state = model.state()
    state.joint_q.assign(q)
    newton.eval_fk(model,state.joint_q,state.joint_qd,state)
    poses = state.body_q.numpy()
    report['native_vs_cpu_fk'] = {}
    for short,name in [('flange','elfin_end_link'),('tcp','suction_contact_frame')]:
        actual = poses[model.body_label.index('S20/'+name)]
        report['native_vs_cpu_fk'][short] = dict(position_error_m=float(np.linalg.norm(actual[:3]-world[short][:3,3])),
            orientation_error_deg=float(np.degrees((R.from_quat(actual[3:]).inv()*R.from_matrix(world[short][:3,:3])).magnitude())))
    viewer = newton.viewer.ViewerGL(width=1080,height=1440,headless=True)
    viewer.set_model(model)
    viewer.show_ui = False
    starts,ends,colors = [],[],[]
    frame_colors = {'base':(1,.75,.1),'flange':(1,.2,.7),'tcp':(.1,1,.9),'uld':(.4,.6,1)}
    frame_colors.update(world=(.9,.9,.9),pedestal=(.8,.6,.3),camera=(.5,1,.5),opening=(1,.6,.3))
    frame_colors.update(optical=(.8,.6,1),lidar=(1,.85,.5))
    for name,t in world.items():
        for k,col in enumerate(((1,.15,.1),(.15,1,.2),(.2,.45,1))):
            starts.append(t[:3,3]);ends.append(t[:3,3]+t[:3,k]*(.35 if name in ('base','uld') else .16));colors.append(col)
    line_args = [wp.array(np.array(a,dtype='float32'),dtype=wp.vec3) for a in (starts,ends,colors)]
    def frame(angle):
        target = np.array([-.65,.45,1.05])
        pos = target+np.array([5.2*np.cos(angle),5.2*np.sin(angle),2.1])
        delta = target-pos
        viewer.set_camera(pos=wp.vec3(*pos),pitch=np.degrees(np.arctan2(delta[2],np.linalg.norm(delta[:2]))),yaw=np.degrees(np.arctan2(delta[1],delta[0])))
        for _ in range(2):
            viewer.begin_frame(0.)
            viewer.log_state(state)
            viewer.log_lines('frame_axes_X_red_Y_green_Z_blue',*line_args)
            for name,color in frame_colors.items():
                viewer.log_points(name,wp.array([world[name][:3,3]],dtype=wp.vec3),radii=.025,
                                  colors=wp.array([color],dtype=wp.vec3))
            viewer.end_frame()
        return viewer.get_frame().numpy()
    from PIL import Image,ImageDraw,ImageFont
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
    def annotate(pixels):
        image = Image.fromarray(pixels)
        draw = ImageDraw.Draw(image)
        draw.rectangle((0,0,1080,74),fill=(20,24,28))
        draw.text((18,7),'Measured snapshot - EMPTY S20 - current site TF/tool',fill='white',font=font)
        draw.text((18,30),'Projected origins; native axes RGB = XYZ; labels may mark occluded origins',fill='white',font=font)
        draw.text((18,52),'Static FK; no live feedback or independent calibration',fill='white',font=font)
        view = np.array(viewer.camera.get_view_matrix()).reshape(4,4,order='F')
        projection = np.array(viewer.camera.get_projection_matrix()).reshape(4,4,order='F')
        offsets = {'base':(-160,-35),'flange':(85,-30),'tcp':(80,50),'uld':(90,-55),
                   'world':(-110,60),'pedestal':(-180,95),'camera':(-190,-70),'opening':(30,80),
                   'optical':(-190,-110),'lidar':(100,100)}
        for name,t in world.items():
            clip = projection@view@np.r_[t[:3,3],1.]
            ndc = clip[:2]/clip[3]
            x,y = (ndc[0]+1)*540,(1-ndc[1])*720
            dx,dy = offsets[name]
            tx,ty = np.clip(x+dx,12,930),np.clip(y+dy,85,1400)
            color = tuple(int(c*255) for c in frame_colors[name])
            draw.line([(x,y),(tx,ty)],fill=color,width=2)
            draw.ellipse((x-5,y-5,x+5,y+5),fill=color)
            bounds = draw.textbbox((tx,ty),name.upper(),font=font)
            draw.rectangle((bounds[0]-4,bounds[1]-3,bounds[2]+4,bounds[3]+3),fill=(20,24,28))
            draw.text((tx,ty),name.upper(),fill=color,font=font)
        if args.xyz_panel:
            combined=Image.new('RGB',(1680,1440),(20,24,28))
            combined.paste(image,(0,0));table=ImageDraw.Draw(combined)
            table.text((1100,20),'FRAME ORIGINS / metres',fill='white',font=font)
            table.text((1100,48),'Red X / Green Y / Blue Z',fill='white',font=font)
            table.text((1100,76),'W: world    B: robot base    U: ULD',fill='white',font=font)
            for row,(name,t) in enumerate(world.items()):
                y=120+row*116
                table.text((1100,y),name.upper(),fill=tuple(int(c*255) for c in frame_colors[name]),font=font)
                for line,(basis,matrix) in enumerate((('W',t),('B',np.linalg.inv(world['base'])@t),
                                                     ('U',np.linalg.inv(world['uld'])@t))):
                    xyz='  '.join(f'{v:+.4f}' for v in matrix[:3,3])
                    table.text((1100,y+28+line*26),f'{basis} XYZ  {xyz}',fill='white',font=font)
            table.text((1100,1340),'CAMERA/OPTICAL use current site URDF.',fill='white',font=font)
            table.text((1100,1368),'D555 live optical TF used.' if args.camera_driver_tf else 'Live driver optical extrinsics not checked here.',fill='white',font=font)
            table.text((1100,1396),'Static measured-q FK; no robot motion.',fill='white',font=font)
            image=combined
        return image
    try:
        portrait = annotate(frame(.85))
        portrait.save(args.output/'measured_pose_native.png')
        report.update(render_executed=True,render_backend='native Newton ViewerGL headless',
                      pose_static=True,portrait=str(args.output/'measured_pose_native.png'),
                      projected_origin_labels_may_show_occluded_frames=True,
                      body_labels=model.body_label,drawn_frame_origins_world_m={n:t[:3,3].tolist() for n,t in world.items()})
        if args.orbit_frames:
            encoder = subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s',f'{portrait.width}x{portrait.height}',
              '-r','8','-i','-','-c:v','libx264','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(args.output/'measured_pose_orbit.mp4')],stdin=subprocess.PIPE)
            for angle in np.linspace(.45,2.,args.orbit_frames):
                encoder.stdin.write(np.asarray(annotate(frame(angle))).tobytes())
            encoder.stdin.close()
            if encoder.wait():
                raise RuntimeError('ffmpeg failed')
            report['orbit_note'] = 'camera moves around a single static measured pose; no robot motion'
    finally:
        viewer.close()


def main():
    here = Path(__file__).resolve().parent
    root = here.parents[2]
    upstream = root/'reproductions/ludan_greedy_20261023'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pose-json',type=Path,required=True)
    parser.add_argument('--scene',type=Path,default=upstream/'deployment_ws/config/scene_tf.loading_site.yaml')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--render',action='store_true')
    parser.add_argument('--orbit-frames',type=int,default=0)
    parser.add_argument('--include-site-frames',action='store_true')
    parser.add_argument('--xyz-panel',action='store_true')
    parser.add_argument('--site-urdf',type=Path,help='Current site xacro output; keep old default constructor unchanged')
    parser.add_argument('--camera-driver-tf',type=Path,help='Recorded actual D555 driver static transforms')
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    for key,sub in [('WARP_CACHE_PATH','warp_cache'),('XDG_CACHE_HOME','cache'),('MPLCONFIGDIR','mpl_config')]:
        os.environ[key] = str(args.output/sub)
    os.environ['PYOPENGL_USE_ACCELERATE'] = '0'
    policy,vendor = upstream/'policy',upstream/'policy/vendor/roboticarm'
    xml = site_empty_urdf(args.site_urdf,upstream) if args.site_urdf else original_empty_urdf(policy,vendor)
    (args.output/'empty_robot_original_mount.urdf').write_text(ET.tostring(xml,encoding='unicode'))
    cfg = yaml.safe_load(args.scene.read_text())
    pose=json.loads(args.pose_json.read_text())
    if 'flange_pose_raw' in pose:
        pose=dict(pose,act_pos_raw=pose['flange_pose_raw'],tcp_config_raw=pose['tcp_config'],
                  ucs_config_raw=pose['ucs_config'])
    driver_optical=None
    if args.camera_driver_tf:
        path=root/'experiments/s20_online_mixed8_20261024/deployment/received_to_observation.py'
        import importlib.util
        spec=importlib.util.spec_from_file_location('received_camera_tf',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        capture=json.loads(args.camera_driver_tf.read_text())
        driver_optical=module.driver_transform(capture['driver_static_transforms'],'d555_check_link',capture['paired']['depth_frame'])
    q,world,report = analyze(pose,xml,cfg,args.include_site_frames,driver_optical)
    report['camera_optical_source']=str(args.camera_driver_tf) if args.camera_driver_tf else 'site URDF default'
    report.update(pose_source=str(args.pose_json.resolve()),scene_source=str(args.scene.resolve()))
    output = args.output/'comparison.json'
    output.write_text(json.dumps(report,indent=2))
    if args.render:
        render(args,policy,vendor,xml,q,world,report)
        output.write_text(json.dumps(report,indent=2))
    print(json.dumps(dict(output=str(output),controller_errors=report['controller_comparisons'],rendered=report['render_executed'])))


if __name__ == '__main__':
    main()
