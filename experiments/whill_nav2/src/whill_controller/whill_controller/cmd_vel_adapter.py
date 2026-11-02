"""Standard cmd_vel input adapter; never generates or repeats velocity commands."""
from copy import deepcopy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from geometry_msgs.msg import Twist,TwistStamped

class CmdVelAdapter(Node):
    def __init__(self):
        super().__init__('whill_cmd_vel_adapter')
        topic=self.declare_parameter('cmd_vel_topic','/cmd_vel').value
        output=self.declare_parameter('output_topic','/whill/controller/navigation').value
        self.frame=self.declare_parameter('base_frame','base_link').value
        stamped=self.declare_parameter('stamped_input',False).value
        self.publisher=self.create_publisher(TwistStamped,output,1)
        self.subscription=self.create_subscription(TwistStamped if stamped else Twist,topic,
            self.stamped if stamped else self.unstamped,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.get_logger().info(f'{topic} -> {output}; no command heartbeat or replay')
    def unstamped(self,msg):
        out=TwistStamped();out.header.stamp=self.get_clock().now().to_msg();out.header.frame_id=self.frame
        out.twist=deepcopy(msg);self.publisher.publish(out)
    def stamped(self,msg):
        # Preserve source time and frame so the actuator can reject stale input.
        self.publisher.publish(deepcopy(msg))

def main():
    rclpy.init();node=CmdVelAdapter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
