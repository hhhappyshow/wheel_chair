"""Native Newton saved-FK viewer. No online planning, dynamics or robot IO."""
import argparse
from pathlib import Path
import time
import numpy as np
import yaml
from compare import UPSTREAM,SCENE,p
from online_packing_newton import make_model
import newton
import newton.viewer
import warp as wp


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-folder",type=Path,default=Path(__file__).resolve().parent/"straight_stack_committed_final")
    parser.add_argument("--seconds",type=float,default=0.)
    args=parser.parse_args();wp.set_device("cuda:0")
    hull=p.Hull.from_config(yaml.safe_load(SCENE.read_text()))
    model,_,_,_=make_model(dict(container=hull.as_dict(),steps=[dict(incoming=dict(id=f"B{i+1}",dims=[.55,.4,.25])) for i in range(3)]),UPSTREAM/"policy/vendor/roboticarm",SCENE)
    data=np.load(args.run_folder/"trajectory.npz");state=model.state()
    viewer=newton.viewer.ViewerGL(width=1280,height=800);viewer.set_model(model);viewer.show_ui=True
    viewer.set_camera(pos=wp.vec3(3.5,-4.,2.7),pitch=-12.,yaw=138.)
    start=time.perf_counter();frame=0
    try:
        while viewer.is_running() and (not args.seconds or time.perf_counter()-start<args.seconds):
            tick=time.perf_counter()
            if viewer.should_step():
                frame=(frame+1)%len(data["joint_q"])
            state.body_q.assign(data["body_q"][frame]);state.joint_q.assign(data["joint_q"][frame])
            viewer.renderer.set_title("S20 saved kinematic reference | "+str(data["stage"][frame])+" | NOT live physics")
            viewer.begin_frame(frame*.02);viewer.log_state(state);viewer.end_frame()
            time.sleep(max(0.,.02-(time.perf_counter()-tick)))
    finally:viewer.close()


if __name__=="__main__":main()
