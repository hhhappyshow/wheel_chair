"""Native RGB-D current-arrival box modeling; known workspace, no GT mask.

Source dimensions/yaw instantiate the simulated scene only. The planner uses
the measured rectangle/ground height returned by capture_model.
"""
import json
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from observed_cycle import Runtime


class MixedPerception:
    def __init__(self,catalog,out):
        self.catalog=catalog;self.root=Path(out);self.root.mkdir()
        self.runtime=Runtime(self.root,False,catalog=catalog)

    def observe(self,index,yaw_deg):
        r=self.runtime;out=self.root/f"B{index+1}";out.mkdir();r.out=out;r.capture_count=0
        for body in r.boxes:r.fixed[body]=np.array([100.,100.,100.,0,0,0,1.])
        dims=self.catalog[index]
        r.fixed[r.boxes[index]]=np.r_[[0.,1.6,dims[2]/2],R.from_euler("z",yaw_deg,degrees=True).as_quat()]
        r.collision.current_source=index;r.w.held=None;r.frame(r.initial_q,"OBSERVE")
        r.model.bvh_build_shapes(r.state)
        start=time.perf_counter();views=[]
        for _ in range(3):
            observed=r.capture_model();views.append(observed)
            if not observed["image_border_clipped"]:break
            tcp=r.state.body_q.numpy()[r.tcp]
            delta=np.array(observed["top_center_world_m"])-observed["image_center_on_top_plane_world"];delta[2]=.20
            toward=-tcp[:2]/(np.linalg.norm(tcp[:2])+1e-12)
            delta[:2]+=.12*toward
            r.move([(tcp[:3]+delta,tcp[3:])],"ADJUST_VIEW",2.)
            r.model.bvh_build_shapes(r.state)
        if observed["image_border_clipped"]:raise RuntimeError(f"B{index+1}: selected surface still clipped")
        import cv2
        origin=np.array(observed["camera_gl_origin"]);rotation=R.from_quat(observed["camera_gl_xyzw"])
        def project(points):
            xyz=rotation.inv().apply(np.asarray(points)-origin)
            return np.rint(np.c_[319.5+322.8994*xyz[:,0]/(-xyz[:,2]),179.5-322.8994*xyz[:,1]/(-xyz[:,2])]).astype(np.int32)
        image=cv2.imread(str(out/"camera_rgb.png"));center=np.array(observed["top_center_world_m"])
        corners=project(observed["top_corners_world_m"])
        cv2.polylines(image,[corners],True,(255,220,0),2)
        for axis,length,color in ((observed["long_axis_world"],observed["dimensions_m"][0],(30,50,255)),(observed["short_axis_world"],observed["dimensions_m"][1],(50,220,40))):
            endpoints=project([center,center+np.array(axis)*length*.4])
            cv2.arrowedLine(image,tuple(endpoints[0]),tuple(endpoints[1]),color,2,tipLength=.15)
        cv2.imwrite(str(out/"orientation_overlay.png"),image)
        result=dict(index=index+1,observed=observed,views=views,wall_s=time.perf_counter()-start,
                    simulation_source_dimensions_only=dims,simulation_source_yaw_deg=yaw_deg,
                    scope=__doc__,complete_view=True)
        (out/"result.json").write_text(json.dumps(result,indent=2))
        return result
