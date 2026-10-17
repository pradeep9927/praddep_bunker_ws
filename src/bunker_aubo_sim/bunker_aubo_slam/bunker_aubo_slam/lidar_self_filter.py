"""Remove the robot's own arm from the LiDAR cloud before it reaches RTAB-Map.

    /lidar/points (PointCloud2) --> lidar_self_filter --> /lidar/points_filtered

The VLP-16 sits on the front mast. When the arm is out of the stowed pose (ready, inspect, or
moved by hand) the LiDAR hits the arm, and RTAB-Map writes those hits into the map as an
obstacle that travels with the robot: a smear along the whole path. This node computes where
every arm link is (forward kinematics from /joint_states and /robot_description, the same
collision boxes as bunker_aubo_description/self_collision.py), inflates them by `margin`, and
drops all points inside. Until the URDF and joint states have arrived it passes clouds through
unchanged, so SLAM never stalls because of this node.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import String

from bunker_aubo_description.self_collision import arm_boxes, points_in_boxes


def xyz_view(msg: PointCloud2):
    """(N,3) float xyz and the raw (N, point_step) bytes of a PointCloud2."""
    off = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)
    dt = '>f4' if msg.is_bigendian else '<f4'
    return np.stack([raw[:, off[c]:off[c] + 4].copy().view(dt).ravel() for c in 'xyz'], axis=1).astype(float), raw


class LidarSelfFilter(Node):
    def __init__(self):
        super().__init__('lidar_self_filter')
        self.margin = float(self.declare_parameter('margin', 0.06).value)
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.checker = None
        self.T_base_lidar = None
        self.lidar_frame = None
        self.joints = {}
        self.removed = 0
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self.on_urdf, latched)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 20)
        self.pub = self.create_publisher(PointCloud2, 'points_out', qos_profile_sensor_data)
        self.create_subscription(PointCloud2, 'points_in', self.on_cloud, qos_profile_sensor_data)
        self.create_timer(10.0, self.report)

    def on_urdf(self, msg: String):
        try:
            from bunker_aubo_description.self_collision import SelfCollisionChecker
            self.checker = SelfCollisionChecker(msg.data)
            self.get_logger().info(f'arm self-filter ready ({len(self.checker.moving)} arm links, '
                                   f'margin {self.margin:.2f} m)')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'cannot build the arm model, passing clouds through: {exc}')
            self.checker = None

    def on_joints(self, msg: JointState):
        self.joints.update(zip(msg.name, msg.position))

    def on_cloud(self, msg: PointCloud2):
        if self.checker is None or not self.joints:
            self.pub.publish(msg)
            return
        m = self.checker.model
        if self.lidar_frame != msg.header.frame_id:
            if msg.header.frame_id not in m.links:
                self.pub.publish(msg)
                return
            self.lidar_frame = msg.header.frame_id
            self.T_base_lidar = m.link_pose(self.lidar_frame)      # fixed to the chassis
        R, c, h = arm_boxes(self.checker, self.joints, self.T_base_lidar, self.margin)
        pts, raw = xyz_view(msg)
        drop = points_in_boxes(pts, R, c, h)
        if not drop.any():
            self.pub.publish(msg)
            return
        keep = raw[~drop]
        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = int(keep.shape[0])
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.point_step * out.width
        out.is_dense = msg.is_dense
        out.data = keep.tobytes()
        self.removed += int(drop.sum())
        self.pub.publish(out)

    def report(self):
        if self.removed:
            self.get_logger().info(f'removed {self.removed} arm points in the last 10 s')
        self.removed = 0


def main(args=None):
    rclpy.init(args=args)
    node = LidarSelfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
