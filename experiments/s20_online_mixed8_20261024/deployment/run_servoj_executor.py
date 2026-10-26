"""Use the author's SDK executor with time-preserving ServoJ slowdown."""
import math


TIMING_POLICY='preserve_input_or_slow_v2'


def subdivide_reference(path_deg,factor):
    import numpy as np
    q=np.asarray(path_deg,dtype=float)
    if factor==1 or len(q)<2:return q.tolist()
    alpha=np.arange(factor,dtype=float)/factor
    out=(q[:-1,None,:]+(q[1:]-q[:-1])[:,None,:]*alpha[None,:,None]).reshape(-1,6)
    return np.vstack((out,q[-1])).tolist()


def preserve_reference_velocity(path_deg,dt_s,max_vel_deg):
    import numpy as np
    if len(path_deg)<2:return [list(q) for q in path_deg]
    q=np.asarray(path_deg,dtype=float)
    peak=float(np.max(np.abs(np.diff(q,axis=0))))/dt_s
    # The author's per-interval ceil produced speed jumps when adjacent
    # intervals received different subdivision counts. Stretch uniformly.
    factor=max(1,math.ceil(peak/max_vel_deg))
    return subdivide_reference(q,factor)


def preserve_reference_braking(path_deg,dt_s,max_decel_deg):
    import numpy as np
    if len(path_deg)<2:
        return [list(q) for q in path_deg]
    q=np.asarray(path_deg,dtype=float)
    velocity=np.diff(q,axis=0)/dt_s
    acceleration=np.diff(np.vstack((np.zeros((1,6)),velocity,np.zeros((1,6)))),axis=0)/dt_s
    # Subdivide every original interval equally: no knot is skipped, no dwell
    # removed and no interval shortened. At original knots, discrete accel
    # decreases linearly with this subdivision factor.
    factor=max(1,math.ceil(float(np.max(np.abs(acceleration)))/max_decel_deg))
    return subdivide_reference(q,factor)


def main():
    from elfin_trajectory_executor import huayan_interface as interface
    from elfin_trajectory_executor import trajectory_executor_node as executor
    interface.limit_servo_braking=preserve_reference_braking
    interface.densify_servo_j_path=preserve_reference_velocity
    original_node=executor.TrajectoryExecutorNode

    class TimingPreservingNode(original_node):
        def __init__(self):
            super().__init__()
            self.declare_parameter('servoj_timing_policy',TIMING_POLICY)
            self.get_logger().info('ServoJ timing policy: '+TIMING_POLICY)

    executor.TrajectoryExecutorNode=TimingPreservingNode
    executor.main()


if __name__=='__main__':main()
