"""Send Nav2 goals from the command line.

    ros2 run bunker_aubo_navigation go_to north_corridor      # named goal (config/goals.yaml)
    ros2 run bunker_aubo_navigation go_to 2.0 -1.5 90         # x y yaw[deg] in the map frame
    ros2 run bunker_aubo_navigation go_to --patrol [--laps 2] # outer corridor lap(s)
    ros2 run bunker_aubo_navigation go_to --list

Ctrl+C cancels the running goal.
"""
import argparse
import math
import os
import sys

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from .goals import load_goals, world_to_map, yaw_to_quaternion

STATUS = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_ABORTED: 'aborted',
          GoalStatus.STATUS_CANCELED: 'canceled'}


class GoTo(Node):
    def __init__(self):
        super().__init__('go_to')
        self.to_pose = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.through = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

    def pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        q = yaw_to_quaternion(yaw)
        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = q
        return p

    def run(self, client, goal, label):
        if not client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('Nav2 is not running (no navigate_to_pose action server)')
            return False
        self.get_logger().info(f'going to {label}')
        send = client.send_goal_async(goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('goal rejected')
            return False
        self._last = 0.0
        result = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result)
        except KeyboardInterrupt:
            self.get_logger().info('cancelling goal')
            rclpy.spin_until_future_complete(self, handle.cancel_goal_async(), timeout_sec=3.0)
            raise
        status = result.result().status
        self.get_logger().info(f'{label}: {STATUS.get(status, status)}')
        return status == GoalStatus.STATUS_SUCCEEDED

    def on_feedback(self, msg):
        fb = msg.feedback
        t = fb.navigation_time.sec + fb.navigation_time.nanosec * 1e-9
        if t - self._last >= 2.0:
            self._last = t
            self.get_logger().info(f'  {fb.distance_remaining:.1f} m to go, '
                                   f'{fb.number_of_recoveries} recoveries, {t:.0f} s')


def main(argv=None):
    argv = remove_ros_args(sys.argv if argv is None else argv)[1:]
    p = argparse.ArgumentParser(description='Send Nav2 goals (named, x y yaw, or a patrol).')
    p.add_argument('target', nargs='*', help='goal name, or x y [yaw_deg] in the map frame')
    p.add_argument('--patrol', action='store_true', help='drive the patrol list from goals.yaml')
    p.add_argument('--laps', type=int, default=1)
    p.add_argument('--list', action='store_true', help='print the named goals and exit')
    p.add_argument('--goals', default=os.path.join(
        get_package_share_directory('bunker_aubo_navigation'), 'config', 'goals.yaml'))
    a = p.parse_args(argv)

    spawn, goals, patrol = load_goals(a.goals)
    if a.list:
        for name, g in goals.items():
            m = world_to_map(g, spawn)
            print(f'{name:16s} world ({g[0]:6.2f}, {g[1]:6.2f})  map ({m[0]:6.2f}, {m[1]:6.2f}, '
                  f'{math.degrees(m[2]):5.0f} deg)')
        print('patrol:', ' -> '.join(patrol))
        return

    rclpy.init()
    node = GoTo()
    ok = True
    try:
        if a.patrol:
            for lap in range(a.laps):
                goal = NavigateThroughPoses.Goal()
                goal.poses = [node.pose(*world_to_map(goals[n], spawn)) for n in patrol]
                ok = node.run(node.through, goal, f'patrol lap {lap + 1}/{a.laps}') and ok
        elif len(a.target) == 1 and a.target[0] in goals:
            goal = NavigateToPose.Goal()
            goal.pose = node.pose(*world_to_map(goals[a.target[0]], spawn))
            ok = node.run(node.to_pose, goal, a.target[0])
        elif len(a.target) in (2, 3):
            x, y = float(a.target[0]), float(a.target[1])
            yaw = math.radians(float(a.target[2])) if len(a.target) == 3 else 0.0
            goal = NavigateToPose.Goal()
            goal.pose = node.pose(x, y, yaw)
            ok = node.run(node.to_pose, goal, f'({x:.2f}, {y:.2f}) in map')
        else:
            p.error(f'unknown goal {" ".join(a.target)!r}; names: {", ".join(goals)}')
    except KeyboardInterrupt:
        ok = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
