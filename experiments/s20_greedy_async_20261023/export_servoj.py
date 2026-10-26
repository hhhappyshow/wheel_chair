"""Export the checked reference to ServoJ degree/time arrays, without SDK IO."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
from compare import UPSTREAM

sys.path.insert(0,str(UPSTREAM/"deployment_ws/src/elfin_trajectory_executor"))
from elfin_trajectory_executor.servo_j import fjt_to_servo_j_deg


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run",type=Path,required=True)
    args=parser.parse_args()
    data=np.load(args.run/"trajectory.npz")
    stages=data["stage"];joint=data["joint_q"][:,:6]
    split=np.r_[0,np.flatnonzero(stages[1:]!=stages[:-1])+1,len(stages)]
    streams=[]
    for start,stop in zip(split,split[1:]):
        q=joint[start:stop];ts=np.arange(len(q))*.02
        points,error=fjt_to_servo_j_deg(q.tolist(),ts.tolist(),dt_s=.02)
        if error:raise ValueError(error)
        streams.append(dict(phase=str(stages[start]),servo_time_s=.02,positions_deg=points,
                            duration_s=float(ts[-1]),start_q_rad=q[0].tolist(),
                            event_after="release" if "insert" in str(stages[start]) else "next_object"))
    output=args.run/"servoj_reference.json"
    output.write_text(json.dumps(dict(execution_enabled=False,
        scope="simulation reference only; independent entry initialization per box; no real start-state binding",
        converter="classmate fjt_to_servo_j_deg; SDK would use StartServo/PushServoJ; neither called here",
        extra_velocity_or_acceleration_caps_added=False,streams=streams),indent=2))
    print(output)


if __name__=="__main__":main()
