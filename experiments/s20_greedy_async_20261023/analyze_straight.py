"""Measure actual FK geometry, without introducing a motion-speed gate."""
import argparse
import json
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R
from compare import SCENE,p
from scene_frames import container_pose,rotation,transform_point


def distance_to_polyline(points,line):
    a,b=line[:-1],line[1:];v=b-a
    t=np.clip(np.sum((points[:,None]-a)*v,axis=2)/np.sum(v*v,axis=1),0,1)
    return np.linalg.norm(points[:,None]-(a+t[:,:,None]*v),axis=2).min(axis=1)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--run-folder",type=Path,required=True)
    args=parser.parse_args();report=json.loads((args.run_folder/"result.json").read_text());data=np.load(args.run_folder/"trajectory.npz")
    cfg=yaml.safe_load(SCENE.read_text());hull=p.Hull.from_config(cfg);ct=container_pose(cfg)
    tcp=list(data["body_labels"]).index("S20/suction_contact_frame");results=[]
    for cycle in report["cycles"]:
        if cycle["status"]!="kinematic_entry_place_return":continue
        n=cycle["arrival"];dims=np.array(cycle["box"]["dims"])
        def world(points):
            local=np.array(points)+dims/2+[hull.door_x,hull.inner_y0,hull.floor_z];local[:,2]+=dims[2]/2
            return transform_point(ct,local)
        loaded=np.flatnonzero(data["stage"]==f"B{n}_insert");empty=np.flatnonzero(data["stage"]==f"B{n}_empty_return")
        error=distance_to_polyline(data["body_q"][loaded,tcp,:3],world(cycle["route"]))
        line=[cycle["route"][-1],cycle["route"][0]]
        if cycle["return_mode"]=="fresh_empty_cartesian":
            line=next(a["route"] for a in cycle["return_attempts"] if a["error"] is None)
        empty_error=distance_to_polyline(data["body_q"][empty,tcp,:3],world(line))
        results.append(dict(arrival=n,loaded_polyline_error_max_m=float(error.max()),loaded_polyline_error_rms_m=float(np.sqrt(np.mean(error**2))),
                            empty_line_error_max_m=float(empty_error.max()),release_to_empty_q_jump_rad=float(abs(data["joint_q"][loaded[-1]]-data["joint_q"][empty[0]]).max()),
                            empty_joint_travel_reduction_fraction=1-cycle["empty_joint_travel_rad"]/cycle["loaded_joint_travel_rad"]))
    summary=dict(scope=__doc__,results=results,inter_box_entry_initialization="independent; no common loaded-staging connector proved",natural_release=False)
    (args.run_folder/"straight_geometry.json").write_text(json.dumps(summary,indent=2));print(json.dumps(summary))


if __name__=="__main__":main()
