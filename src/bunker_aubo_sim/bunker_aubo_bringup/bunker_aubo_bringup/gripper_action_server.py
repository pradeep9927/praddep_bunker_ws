"""GripperCommand action server for the two-finger gripper.

GripperCommand.position is the finger gap in metres (0 = closed, max_gap = fully open). Each
goal is turned into a FollowJointTrajectory goal for gripper_controller (each finger moves to
gap / 2). The server then watches /joint_states until the fingers reach the gap, or stop
early (an object between the fingers), or a timeout passes.

Only the newest goal is executed: a new goal (e.g. pressing Open while it is still closing)
preempts the running one, so the result the GUI shows always belongs to the last command.
Timeouts use the node clock, so they stay correct when Gazebo runs slower than real time.
"""
import math
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from .gripper_logic import HOLDING, REACHED, clamp_gap, classify, move_duration

WALL_LIMIT = 30.0   # s, last-resort guard if /clock stops


class GripperActionServer(Node):
    def __init__(self):
        super().__init__('gripper_action_server')
        self.joints = self.declare_parameter(
            'joints', ['gripper_left_finger_joint', 'gripper_right_finger_joint']).value
        self.max_gap = float(self.declare_parameter('max_gap', 0.08).value)
        self.speed = float(self.declare_parameter('finger_speed', 0.08).value)        # m/s per finger
        self.tolerance = float(self.declare_parameter('goal_tolerance', 0.003).value)  # m on the gap
        self.settle_timeout = float(self.declare_parameter('settle_timeout', 3.0).value)  # s after the move
        self.still_velocity = float(self.declare_parameter('still_velocity', 0.002).value)  # m/s
        self.still_time = float(self.declare_parameter('still_time', 0.3).value)        # s
        traj_action = self.declare_parameter(
            'trajectory_action', '/gripper_controller/follow_joint_trajectory').value

        cb = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.positions = {}
        self.velocities = {}
        self.generation = 0          # increases with every accepted goal
        self.goal_generation = {}    # goal id -> generation it was accepted with
        self.active_traj = None
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10,
                                 callback_group=cb)
        self.client = ActionClient(self, FollowJointTrajectory, traj_action, callback_group=cb)
        self.server = ActionServer(
            self, GripperCommand, '/gripper_controller/gripper_cmd',
            execute_callback=self.execute, goal_callback=self.on_goal,
            handle_accepted_callback=self.on_accepted,
            cancel_callback=self.on_cancel, callback_group=cb)
        self.get_logger().info('GripperCommand server ready on /gripper_controller/gripper_cmd')

    # ------------------------------------------------------------------ state
    def on_joint_states(self, msg: JointState):
        with self.lock:
            for i, name in enumerate(msg.name):
                if name in self.joints:
                    self.positions[name] = msg.position[i]
                    if i < len(msg.velocity):
                        self.velocities[name] = msg.velocity[i]

    def current_gap(self):
        with self.lock:
            if not all(j in self.positions for j in self.joints):
                return float('nan')
            return sum(self.positions[j] for j in self.joints)

    def fingers_still(self):
        with self.lock:
            return all(abs(self.velocities.get(j, 0.0)) < self.still_velocity for j in self.joints)

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ goals
    def on_goal(self, goal):
        if math.isnan(goal.command.position):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_accepted(self, goal_handle):
        with self.lock:
            self.generation += 1   # newer goals preempt this one
            self.goal_generation[bytes(goal_handle.goal_id.uuid)] = self.generation
        goal_handle.execute()

    def on_cancel(self, _goal_handle):
        traj = self.active_traj
        if traj is not None:
            traj.cancel_goal_async()
        return CancelResponse.ACCEPT

    def superseded(self, goal_handle):
        with self.lock:
            return self.goal_generation.get(bytes(goal_handle.goal_id.uuid)) != self.generation

    def finish(self, goal_handle, gap, start, outcome):
        with self.lock:
            self.goal_generation.pop(bytes(goal_handle.goal_id.uuid), None)
        result = GripperCommand.Result()
        final = self.current_gap()
        result.position = final
        result.effort = 0.0
        kind = classify(start, gap, final, self.tolerance)
        result.reached_goal = kind == REACHED
        result.stalled = kind == HOLDING
        if outcome == 'canceled':
            goal_handle.canceled()
        elif outcome == 'preempted' or outcome == 'error':
            goal_handle.abort()
        elif kind in (REACHED, HOLDING):
            goal_handle.succeed()
        else:
            goal_handle.abort()
        self.get_logger().info(f'gripper {outcome}/{kind}: gap {final * 1000:.1f} mm '
                               f'(target {gap * 1000:.1f} mm)')
        return result

    def execute(self, goal_handle):
        gap = clamp_gap(goal_handle.request.command.position, self.max_gap)
        start = self.current_gap()

        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('gripper_controller action server not available')
            return self.finish(goal_handle, gap, start, 'error')
        if self.superseded(goal_handle):
            return self.finish(goal_handle, gap, start, 'preempted')

        duration = move_duration(start, gap, self.speed, self.max_gap)
        traj_goal = FollowJointTrajectory.Goal()
        traj_goal.trajectory.joint_names = list(self.joints)
        point = JointTrajectoryPoint()
        point.positions = [gap / 2.0] * len(self.joints)
        point.velocities = [0.0] * len(self.joints)
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1.0) * 1e9))
        traj_goal.trajectory.points = [point]

        # a new trajectory goal replaces the previous one inside gripper_controller
        send = self.client.send_goal_async(traj_goal)
        wall_end = time.monotonic() + 5.0
        while not send.done() and time.monotonic() < wall_end:
            time.sleep(0.01)
        handle = send.result() if send.done() else None
        if handle is None or not handle.accepted:
            self.get_logger().warn('gripper trajectory not accepted by gripper_controller')
            return self.finish(goal_handle, gap, start, 'error')
        self.active_traj = handle

        # watch the fingers (sim time) instead of trusting the trajectory result alone
        t0 = self.sim_now()
        wall_end = time.monotonic() + WALL_LIMIT
        still_since = None
        outcome = 'done'
        while True:
            if goal_handle.is_cancel_requested:
                handle.cancel_goal_async()
                outcome = 'canceled'
                break
            if self.superseded(goal_handle):
                outcome = 'preempted'
                break
            elapsed = self.sim_now() - t0
            cur = self.current_gap()
            if not math.isnan(cur) and abs(cur - gap) <= self.tolerance:
                break
            if elapsed > duration:                     # move should be over: wait until still
                if self.fingers_still():
                    still_since = elapsed if still_since is None else still_since
                    if elapsed - still_since >= self.still_time:
                        break
                else:
                    still_since = None
            if elapsed > duration + self.settle_timeout or time.monotonic() > wall_end:
                break
            fb = GripperCommand.Feedback()
            fb.position = cur
            goal_handle.publish_feedback(fb)
            time.sleep(0.02)

        if self.active_traj is handle:
            self.active_traj = None
        return self.finish(goal_handle, gap, start, outcome)


def main(args=None):
    rclpy.init(args=args)
    node = GripperActionServer()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
