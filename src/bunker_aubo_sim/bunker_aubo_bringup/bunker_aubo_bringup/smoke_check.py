import math
import struct
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, PointCloud2
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint',
              'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
ALL_JOINTS = ARM_JOINTS + ['gripper_left_finger_joint', 'gripper_right_finger_joint'] + \
    [f'{s}_wheel_{i}_joint' for s in ('left', 'right') for i in range(1, 5)]

TOPICS = {
    '/joint_states': (JointState, None, 30.0),
    '/odom': (Odometry, 'odom', 20.0),
    '/odometry/filtered': (Odometry, 'odom', 20.0),
    '/imu/data': (Imu, 'imu_link', 100.0),
    '/lidar/points': (PointCloud2, 'lidar_link', 5.0),
    '/base_camera/color/image_raw': (Image, 'base_camera_color_optical_frame', 5.0),
    '/base_camera/depth/image_rect_raw': (Image, 'base_camera_color_optical_frame', 5.0),
    '/base_camera/color/camera_info': (CameraInfo, 'base_camera_color_optical_frame', 5.0),
    '/base_camera/depth/camera_info': (CameraInfo, 'base_camera_color_optical_frame', 5.0),
    '/wrist_camera/color/image_raw': (Image, 'wrist_camera_color_optical_frame', 5.0),
    '/wrist_camera/depth/image_rect_raw': (Image, 'wrist_camera_color_optical_frame', 5.0),
    '/wrist_camera/color/camera_info': (CameraInfo, 'wrist_camera_color_optical_frame', 5.0),
    '/wrist_camera/depth/camera_info': (CameraInfo, 'wrist_camera_color_optical_frame', 5.0),
}


@dataclass
class TopicStats:
    count: int = 0
    first_rx: float = 0.0
    last_rx: float = 0.0
    last_msg: object = None
    stamps: List[float] = field(default_factory=list)


class SmokeCheck(Node):
    def __init__(self):
        super().__init__('smoke_check')
        self.duration = self.declare_parameter('duration', 8.0).value
        self.active = self.declare_parameter('active', False).value
        self.stats: Dict[str, TopicStats] = {t: TopicStats() for t in TOPICS}
        self.clock_time = None
        self.results = []

        for topic, (mtype, _, _) in TOPICS.items():
            self.create_subscription(mtype, topic, lambda m, t=topic: self.on_msg(t, m),
                                     qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_clock(self, msg):
        self.clock_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_msg(self, topic, msg):
        s = self.stats[topic]
        now = time.monotonic()
        if s.count == 0:
            s.first_rx = now
        s.count += 1
        s.last_rx = now
        s.last_msg = msg
        hdr = getattr(msg, 'header', None)
        if hdr is not None:
            s.stamps.append(hdr.stamp.sec + hdr.stamp.nanosec * 1e-9)

    def check(self, name, ok, detail=''):
        self.results.append((name, bool(ok), detail))
        mark = 'PASS' if ok else 'FAIL'
        self.get_logger().info(f'[{mark}] {name} {detail}')

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_future(self, future, timeout):
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.result() if future.done() else None

    def passive_checks(self):
        self.get_logger().info(f'Collecting data for {self.duration:.0f} s ...')
        self.spin_for(self.duration)

        self.check('/clock is published (sim time)', self.clock_time is not None)

        for topic, (_, frame, min_rate) in TOPICS.items():
            s = self.stats[topic]
            if s.count < 2:
                self.check(f'{topic} publishes', False, f'({s.count} msgs)')
                continue
            rate = (s.count - 1) / max(1e-6, s.last_rx - s.first_rx)
            self.check(f'{topic} rate', rate >= 0.4 * min_rate, f'{rate:.1f} Hz (nominal >= {min_rate})')
            if frame is not None:
                fid = s.last_msg.header.frame_id
                self.check(f'{topic} frame_id', fid == frame, f"'{fid}'")
            if s.stamps and self.clock_time is not None:
                lag = self.clock_time - s.stamps[-1]
                self.check(f'{topic} stamped with sim time', -1.0 < lag < 5.0,
                           f'(clock - stamp = {lag:.3f} s)')

        self.check_imu()
        self.check_lidar()
        for cam in ('base_camera', 'wrist_camera'):
            self.check_camera(cam)
        self.check_joint_states()
        self.check_tf()

    def check_imu(self):
        m = self.stats['/imu/data'].last_msg
        if m is None:
            return
        q = m.orientation
        n = math.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2)
        self.check('IMU quaternion normalised', abs(n - 1.0) < 1e-3, f'|q|={n:.4f}')
        a = m.linear_acceleration
        g = math.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2)
        self.check('IMU measures gravity', 9.0 < g < 10.6 and a.z > 8.5, f'|a|={g:.2f}, az={a.z:.2f}')

    def check_lidar(self):
        m = self.stats['/lidar/points'].last_msg
        if m is None:
            return
        names = [f.name for f in m.fields]
        self.check('LiDAR fields', {'x', 'y', 'z'} <= set(names), str(names))
        offs = {f.name: f.offset for f in m.fields}
        n = m.width * m.height
        zs = []
        step = max(1, n // 2000)
        for i in range(0, n, step):
            base = i * m.point_step
            z = struct.unpack_from('<f', bytes(m.data[base + offs['z']: base + offs['z'] + 4]))[0]
            if math.isfinite(z):
                zs.append(z)
        ok = len(zs) > 50 and min(zs) < -0.2 and max(zs) > 0.2
        self.check('LiDAR cloud is 3D and upright', ok,
                   f'({len(zs)} sampled pts, z in [{min(zs, default=0):.2f}, {max(zs, default=0):.2f}])')

    def check_camera(self, cam):
        rgb = self.stats[f'/{cam}/color/image_raw'].last_msg
        depth = self.stats[f'/{cam}/depth/image_rect_raw'].last_msg
        info = self.stats[f'/{cam}/color/camera_info'].last_msg
        if None in (rgb, depth, info):
            return
        self.check(f'{cam} RGB encoding', rgb.encoding in ('rgb8', 'bgr8'), rgb.encoding)
        self.check(f'{cam} depth encoding', depth.encoding in ('32FC1', '16UC1'), depth.encoding)
        same = (rgb.width, rgb.height) == (depth.width, depth.height) == (info.width, info.height)
        self.check(f'{cam} RGB/depth/info sizes agree', same, f'{rgb.width}x{rgb.height}')
        self.check(f'{cam} intrinsics valid', info.k[0] > 0 and info.k[4] > 0,
                   f'fx={info.k[0]:.1f} fy={info.k[4]:.1f}')

    def check_joint_states(self):
        m = self.stats['/joint_states'].last_msg
        if m is None:
            return
        missing = sorted(set(ALL_JOINTS) - set(m.name))
        self.check('joint_states has all 16 joints', not missing, f'missing={missing}' if missing else '')

    def check_tf(self):
        frames = ['base_link', 'imu_link', 'lidar_link', 'base_camera_color_optical_frame',
                  'wrist_camera_color_optical_frame', 'gripper_tcp', 'left_wheel_1_link']
        for f in frames:
            ok = self.tf_buffer.can_transform('base_footprint', f, Time(), Duration(seconds=1.0))
            self.check(f'TF base_footprint <- {f}', ok)
        ok = self.tf_buffer.can_transform('odom', 'base_footprint', Time(), Duration(seconds=1.0))
        self.check('TF odom <- base_footprint (EKF)', ok)

    def active_checks(self):
        pub = self.create_publisher(Twist, '/cmd_vel', 10)
        start = self.stats['/odom'].last_msg
        cmd = Twist()
        cmd.linear.x = 0.3
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            pub.publish(cmd)
            self.spin_for(0.05)
        pub.publish(Twist())
        self.spin_for(1.0)
        stop = self.stats['/odom'].last_msg
        if start is not None and stop is not None:
            d = math.hypot(stop.pose.pose.position.x - start.pose.pose.position.x,
                           stop.pose.pose.position.y - start.pose.pose.position.y)
            self.check('base drives from /cmd_vel', d > 0.3, f'moved {d:.2f} m')

        arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        ok = arm.wait_for_server(timeout_sec=5.0)
        self.check('arm FollowJointTrajectory server', ok)
        js = self.stats['/joint_states'].last_msg
        if ok and js is not None:
            cur = dict(zip(js.name, js.position))
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ['wrist3_joint']
            p = JointTrajectoryPoint()
            target = cur.get('wrist3_joint', 0.0) + 0.5
            p.positions = [target]
            p.time_from_start.sec = 2
            goal.trajectory.points = [p]
            handle = self.wait_future(arm.send_goal_async(goal), 5.0)
            res = self.wait_future(handle.get_result_async(), 10.0) if handle and handle.accepted else None
            self.spin_for(0.5)
            js = self.stats['/joint_states'].last_msg
            err = abs(dict(zip(js.name, js.position)).get('wrist3_joint', 1e9) - target)
            self.check('arm reaches trajectory goal', res is not None and err < 0.05, f'error={err:.3f} rad')

        grip = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        ok = grip.wait_for_server(timeout_sec=5.0)
        self.check('GripperCommand server', ok)
        if ok:
            for gap in (0.0, 0.08):
                g = GripperCommand.Goal()
                g.command.position = gap
                handle = self.wait_future(grip.send_goal_async(g), 5.0)
                res = self.wait_future(handle.get_result_async(), 10.0) if handle and handle.accepted else None
                pos = res.result.position if res else float('nan')
                self.check(f'gripper -> {gap * 1000:.0f} mm', res is not None and abs(pos - gap) < 0.01,
                           f'measured {pos * 1000:.1f} mm')

    def report(self):
        failed = [r for r in self.results if not r[1]]
        total = len(self.results)
        self.get_logger().info(f'==== {total - len(failed)}/{total} checks passed ====')
        for name, _, detail in failed:
            self.get_logger().error(f'FAILED: {name} {detail}')
        return 0 if not failed else 1


def main(args=None):
    rclpy.init(args=args)
    node = SmokeCheck()
    code = 1
    try:
        node.passive_checks()
        if node.active:
            node.active_checks()
        code = node.report()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
