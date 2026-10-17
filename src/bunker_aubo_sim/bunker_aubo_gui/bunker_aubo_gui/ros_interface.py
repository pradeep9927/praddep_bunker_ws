import math
import threading
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Image, JointState, PointCloud2
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

from .msg_utils import colorize_depth, depth_to_metres, image_to_rgb, pointcloud2_to_xyz
from .topic_monitor import TopicMonitor


def _load_msg_type(type_str: str):
    try:
        from rosidl_runtime_py.utilities import get_message
        return get_message(type_str)
    except Exception:  # noqa: BLE001 - optional packages (e.g. rtabmap_msgs) may be missing
        return None


class Frame:
    __slots__ = ('rgb', 'seq')

    def __init__(self):
        self.rgb: Optional[np.ndarray] = None
        self.seq = 0


class RosInterface:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.node = Node('operator_gui')
        self.monitor = TopicMonitor()
        self._lock = threading.Lock()

        self._frames: Dict[str, Frame] = {}
        self._cloud: Optional[np.ndarray] = None
        self._cloud_seq = 0
        self._cloud_frame = ''
        self._joints: Dict[str, float] = {}
        self._odom_twist: Tuple[float, float] = (0.0, 0.0)
        self._urdf: Optional[str] = None
        self._loop_closures = 0
        self._last_loop_id = 0
        self.arm_status = 'idle'
        self.gripper_status = 'idle'
        self._arm_goal = None
        self._gripper_goal = None
        self._gripper_seq = 0
        self._gripper_target = 0.0

        sensor_cb = ReentrantCallbackGroup()
        self._action_cb = MutuallyExclusiveCallbackGroup()
        n = self.node

        special = {cfg['lidar_topic'], '/joint_states', cfg['base']['odom_topic']}
        for cam in cfg['cameras'].values():
            special |= {cam['color'], cam['depth']}
        for topic, type_str, rate in cfg['status_topics']:
            self.monitor.add(topic, float(rate))
            if topic in special:
                continue
            mtype = _load_msg_type(type_str)
            if mtype is None:
                n.get_logger().warn(f'status: message type {type_str} unavailable, skipping {topic}')
                continue
            qos = qos_profile_sensor_data
            if topic == '/map':
                qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            n.create_subscription(mtype, topic, lambda m, t=topic: self._on_generic(t, m), qos,
                                  callback_group=sensor_cb)

        for key, cam in cfg['cameras'].items():
            self._frames[f'{key}/color'] = Frame()
            self._frames[f'{key}/depth'] = Frame()
            n.create_subscription(Image, cam['color'],
                                  lambda m, k=f'{key}/color', t=cam['color']: self._on_color(k, t, m),
                                  qos_profile_sensor_data, callback_group=sensor_cb)
            n.create_subscription(Image, cam['depth'],
                                  lambda m, k=f'{key}/depth', t=cam['depth']: self._on_depth(k, t, m),
                                  qos_profile_sensor_data, callback_group=sensor_cb)

        n.create_subscription(PointCloud2, cfg['lidar_topic'], self._on_cloud, qos_profile_sensor_data,
                              callback_group=sensor_cb)
        n.create_subscription(JointState, '/joint_states', self._on_joints, qos_profile_sensor_data,
                              callback_group=sensor_cb)
        n.create_subscription(Odometry, cfg['base']['odom_topic'], self._on_odom, qos_profile_sensor_data,
                              callback_group=sensor_cb)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        n.create_subscription(String, '/robot_description', self._on_description, latched,
                              callback_group=sensor_cb)

        self.cmd_pub = n.create_publisher(Twist, cfg['base']['cmd_vel_topic'], 10)
        self.arm_client = ActionClient(n, FollowJointTrajectory, cfg['arm']['action'],
                                       callback_group=self._action_cb)
        self.gripper_client = ActionClient(n, GripperCommand, cfg['gripper']['action'],
                                           callback_group=self._action_cb)

        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(n)
        self._thread = threading.Thread(target=self._spin, daemon=True)

    def start(self):
        self._thread.start()

    def _spin(self):
        try:
            self.executor.spin()
        except Exception as exc:  # noqa: BLE001 - executor shut down while the GUI closes
            if rclpy.ok():
                self.node.get_logger().error(f'executor stopped: {exc}')

    def shutdown(self):
        try:
            self.publish_twist(0.0, 0.0)
        except Exception:  # noqa: BLE001
            pass
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()

    def _on_generic(self, topic, msg):
        self.monitor.record(topic)
        loop_id = getattr(msg, 'loop_closure_id', 0)
        if loop_id:
            with self._lock:
                if loop_id != self._last_loop_id:
                    self._loop_closures += 1
                    self._last_loop_id = loop_id

    def _on_color(self, key, topic, msg):
        self.monitor.record(topic)
        try:
            rgb = image_to_rgb(msg)
        except Exception:  # noqa: BLE001 - a malformed frame must not kill the callback
            rgb = None
        if rgb is not None:
            with self._lock:
                f = self._frames[key]
                f.rgb, f.seq = rgb, f.seq + 1

    def _on_depth(self, key, topic, msg):
        self.monitor.record(topic)
        try:
            d = depth_to_metres(msg)
            rgb = colorize_depth(d) if d is not None else None
        except Exception:  # noqa: BLE001
            rgb = None
        if rgb is not None:
            with self._lock:
                f = self._frames[key]
                f.rgb, f.seq = rgb, f.seq + 1

    def _on_cloud(self, msg):
        self.monitor.record(self.cfg['lidar_topic'])
        try:
            xyz = pointcloud2_to_xyz(msg, int(self.cfg['gui']['lidar_max_points']))
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._cloud, self._cloud_seq = xyz, self._cloud_seq + 1
            self._cloud_frame = msg.header.frame_id

    def _on_joints(self, msg):
        self.monitor.record('/joint_states')
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._joints[name] = pos

    def _on_odom(self, msg):
        self.monitor.record(self.cfg['base']['odom_topic'])
        with self._lock:
            self._odom_twist = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def _on_description(self, msg):
        with self._lock:
            self._urdf = msg.data

    def latest_frame(self, key) -> Tuple[Optional[np.ndarray], int]:
        with self._lock:
            f = self._frames.get(key)
            return (f.rgb, f.seq) if f else (None, 0)

    def latest_cloud(self) -> Tuple[Optional[np.ndarray], int]:
        with self._lock:
            return self._cloud, self._cloud_seq

    def cloud_frame(self) -> str:
        with self._lock:
            return self._cloud_frame

    def all_joint_positions(self) -> Dict[str, float]:
        with self._lock:
            return dict(self._joints)

    def joint_positions(self, names: List[str]) -> Optional[List[float]]:
        with self._lock:
            if not all(n in self._joints for n in names):
                return None
            return [self._joints[n] for n in names]

    def odom_twist(self):
        with self._lock:
            return self._odom_twist

    def robot_description(self) -> Optional[str]:
        with self._lock:
            return self._urdf

    def loop_closures(self) -> int:
        with self._lock:
            return self._loop_closures

    def publish_twist(self, v: float, w: float):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.cmd_pub.publish(t)

    def arm_server_ready(self) -> bool:
        return self.arm_client.server_is_ready()

    def gripper_server_ready(self) -> bool:
        return self.gripper_client.server_is_ready()

    def send_arm_goal(self, joints: List[str], positions: List[float], duration: float) -> bool:
        if not self.arm_client.server_is_ready():
            self.arm_status = 'arm controller not available'
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joints)
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in positions]
        p.velocities = [0.0] * len(joints)
        p.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1.0) * 1e9))
        goal.trajectory.points = [p]
        self.arm_status = f'moving ({duration:.1f} s)'
        fut = self.arm_client.send_goal_async(goal)
        fut.add_done_callback(self._arm_goal_response)
        return True

    def _arm_goal_response(self, fut):
        handle = fut.result()
        if not handle.accepted:
            self.arm_status = 'goal rejected'
            return
        self._arm_goal = handle
        handle.get_result_async().add_done_callback(self._arm_result)

    def _arm_result(self, fut):
        res = fut.result()
        code = res.result.error_code
        self.arm_status = 'reached' if code == FollowJointTrajectory.Result.SUCCESSFUL else \
            f'finished with error {code} {res.result.error_string}'.strip()
        self._arm_goal = None

    def cancel_arm(self):
        if self._arm_goal is not None:
            self._arm_goal.cancel_goal_async()
            self.arm_status = 'cancelled'

    def send_gripper(self, gap: float) -> bool:
        """Send a new gap. A newer command replaces the running one (the server preempts it),
        and results of replaced commands are ignored so the status always shows the last one."""
        if not self.gripper_client.server_is_ready():
            self.gripper_status = 'gripper server not available'
            return False
        g = GripperCommand.Goal()
        g.command.position = float(gap)
        g.command.max_effort = 40.0
        with self._lock:
            self._gripper_seq += 1
            seq = self._gripper_seq
            self._gripper_target = float(gap)
        self.gripper_status = f'moving to {gap * 1000:.0f} mm'
        fut = self.gripper_client.send_goal_async(g)
        fut.add_done_callback(lambda f, seq=seq: self._gripper_goal_response(f, seq))
        return True

    def _gripper_latest(self, seq) -> bool:
        with self._lock:
            return seq == self._gripper_seq

    def _gripper_goal_response(self, fut, seq):
        handle = fut.result()
        if not self._gripper_latest(seq):
            return
        if not handle.accepted:
            self.gripper_status = 'rejected'
            return
        self._gripper_goal = handle
        handle.get_result_async().add_done_callback(lambda f, seq=seq: self._gripper_result(f, seq))

    def _gripper_result(self, fut, seq):
        if not self._gripper_latest(seq):
            return                                   # an older command, already replaced
        res = fut.result()
        r = res.result
        pos = 'unknown' if math.isnan(r.position) else f'{r.position * 1000:.0f} mm'
        target = self._gripper_target * 1000
        if res.status == GoalStatus.STATUS_CANCELED:
            self.gripper_status = f'stopped at {pos}'
        elif r.reached_goal:
            self.gripper_status = f'done ({pos})'
        elif r.stalled:
            self.gripper_status = f'holding object ({pos}, target {target:.0f} mm)'
        else:
            self.gripper_status = f'did not reach {target:.0f} mm (at {pos})'
        self._gripper_goal = None

    def cancel_gripper(self):
        if self._gripper_goal is not None:
            self._gripper_goal.cancel_goal_async()
