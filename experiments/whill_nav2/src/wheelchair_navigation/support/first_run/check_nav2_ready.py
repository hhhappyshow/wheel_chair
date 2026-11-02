#!/usr/bin/env python3
"""Wait for all required lifecycle servers to be active."""
import time
import rclpy
from lifecycle_msgs.srv import GetState

SERVERS = ('planner_server', 'controller_server', 'smoother_server', 'behavior_server',
           'bt_navigator', 'waypoint_follower', 'velocity_smoother', 'collision_monitor')
rclpy.init()
node = rclpy.create_node('shaki_nav2_startup_check')
clients = {name: node.create_client(GetState, f'/{name}/get_state') for name in SERVERS}
deadline = time.monotonic() + 45
states = {}
while time.monotonic() < deadline:
    states = {}
    pending = {}
    for name, client in clients.items():
        if client.service_is_ready():
            pending[name] = client.call_async(GetState.Request())
    end = min(deadline, time.monotonic()+3)
    while pending and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        for name, future in list(pending.items()):
            if future.done():
                try:
                    states[name] = future.result().current_state.label
                except Exception:
                    states[name] = 'error'
                del pending[name]
    if all(states.get(name) == 'active' for name in SERVERS):
        print('Nav2 lifecycle: all active', flush=True)
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(0)
    time.sleep(0.5)
print(f'Nav2 lifecycle not ready: {states}', flush=True)
node.destroy_node()
rclpy.shutdown()
raise SystemExit(1)
