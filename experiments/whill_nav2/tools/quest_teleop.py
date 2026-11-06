#!/usr/bin/env python3
"""Quest 3 遥操作 -> /cmd_vel (方案C, 直发 cmd_vel_adapter).

链路: 右手柄相对位姿 -> (v, omega) -> /cmd_vel (Twist)
      -> whill_cmd_vel_adapter -> /whill/controller/navigation -> ESP32 -> 轮椅

注意: 本节点绕过 Velocity Smoother 与 Collision Monitor,
节点内部自带软件限幅/死区/看门狗, 真机首测请把 MAX_V/MAX_W 调小.

操作:
  A键(上升沿)   : 切换 ENABLED / DISABLED
  右扳机按住    : 虚拟摇杆激活, 手柄位移映射为速度
  右扳机松开    : 立即零速, 摇杆回中
  数据超时300ms : 立即零速(看门狗)
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import xrobotoolkit_sdk as xrt

# ---- 可调参数 ----
TOPIC = '/cmd_vel'
RATE_HZ = 20.0          # 发布频率, 与 Nav2 控制/平滑链一致
DEADZONE = 0.02         # 死区, 手柄位移小于此值视为居中
EXPO = 1.5              # 指数曲线, >1 表示小位移更精细
GAIN_V = 2.0            # 前进增益: v = GAIN_V * dy ** EXPO (m/s, dy 单位 m)
GAIN_W = 3.0            # 转向增益: w = GAIN_W * dx ** EXPO (rad/s, dx 单位 m)
MAX_V = 0.30            # 速度硬限幅, 首测建议调小(链条上限 0.5366)
MAX_W = 0.50            # 角速度硬限幅(链条上限 0.8109)
WATCHDOG_S = 0.3        # 超过此时长无新帧 -> 零速
DEDUP_NANOS = 1         # 时间戳去重容差(ns)


def quat_to_yaw(q):
    """四元数 [x,y,z,w] -> 平面 yaw 角."""
    x, y, z, w = q[0], q[1], q[2], q[3]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class QuestTeleop(Node):
    def __init__(self):
        super().__init__('quest_teleop')
        self.pub = self.create_publisher(Twist, TOPIC, 10)
        self.timer = self.create_timer(1.0 / RATE_HZ, self.tick)

        # 虚拟摇杆状态
        self.origin = None          # P0: 激活瞬间的手柄位姿 (pos, yaw)
        self.last_stamp = None      # 上一帧设备时间戳
        self.last_seen = None       # 最近一次收到新数据的时间(用于看门狗)
        self.enabled = False        # A键总开关
        self.a_prev = False
        self.last_pub = (0.0, 0.0)  # 上一拍输出, 用于日志

        xrt.init()
        self.get_logger().info(f'xrt 初始化完成, 目标话题 {TOPIC}, '
                               f'按 A 键使能, 按住右扳机移动轮椅')

    def reset_stick(self, pose):
        """以当前手柄位姿为新的摇杆中心."""
        p = pose[:3]
        q = pose[3:7]
        self.origin = (p, quat_to_yaw(q))
        self.last_stamp = None

    def tick(self):
        cmd = Twist()
        head = xrt.get_headset_pose()
        right = xrt.get_right_controller_pose()
        stamp = xrt.get_time_stamp_ns()
        a_now = bool(xrt.get_A_button())
        trig = float(xrt.get_right_trigger())

        # --- A 键上升沿: 使能切换 ---
        if a_now and not self.a_prev:
            self.enabled = not self.enabled
            self.get_logger().info(
                f'A键: 遥操作{"使能" if self.enabled else "失能"}')
            if not self.enabled:
                self.origin = None
        self.a_prev = a_now

        # --- 数据有效性: 时间戳去重 + 看门狗 ---
        fresh = stamp != self.last_stamp and stamp != 0
        if fresh:
            self.last_seen = time.monotonic()
        self.last_stamp = stamp
        timed_out = (self.last_seen is None or
                     time.monotonic() - self.last_seen > WATCHDOG_S)
        active = (self.enabled and self.origin is not None
                  and trig > 0.5 and not timed_out)

        if active:
            # 相对位姿: 世界系位移转到 P0 手柄坐标系
            # (x前 y左 z上; 手柄初始朝向定义"往前推"的方向)
            p, yaw0 = self.origin
            dp = (right[0] - p[0], right[1] - p[1])
            c, s = math.cos(yaw0), math.sin(yaw0)
            # 把世界系位移投到 P0 手柄自身坐标系
            # fwd = 手柄"前方"分量(手柄局部 x 轴), left = 手柄"左方"分量(局部 y 轴)
            # 注意: 若真机上发现前后/左右反了, 说明该 SDK 手柄四元数的轴向
            # 与此约定不同, 交换 fwd/left 即可, 勿改映射符号
            fwd = c * dp[0] + s * dp[1]
            left = -s * dp[0] + c * dp[1]

            def shape(d):
                mag = abs(d)
                if mag < DEADZONE:
                    return 0.0
                return math.copysign(mag ** EXPO, d)

            v = max(-MAX_V, min(MAX_V, GAIN_V * shape(fwd)))
            w = max(-MAX_W, min(MAX_W, GAIN_W * shape(left)))  # 左推=左转=+w
            cmd.linear.x = v
            cmd.angular.z = w
        else:
            # 扳机松开/失能/超时 -> 零速并回中
            if self.origin is not None and (not self.enabled or trig <= 0.5):
                self.reset_stick(right)
            if timed_out and self.last_pub != (0.0, 0.0):
                self.get_logger().warn('数据超时, 输出零速', throttle_duration_sec=2.0)

        self.pub.publish(cmd)
        if (cmd.linear.x, cmd.angular.z) != self.last_pub:
            self.last_pub = (cmd.linear.x, cmd.angular.z)
            self.get_logger().info(
                f'v={cmd.linear.x:+.3f} m/s  w={cmd.angular.z:+.3f} rad/s  '
                f'trig={trig:.2f} en={self.enabled} fresh={fresh}')


def main():
    rclpy.init()
    node = QuestTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前补发一拍零速, 防止轮椅带速停止
        zero = Twist()
        node.pub.publish(zero)
        xrt.close()
        node.destroy_node()
        rclpy.try_shutdown()
        print('已退出, 最后输出: 零速')


if __name__ == '__main__':
    main()
