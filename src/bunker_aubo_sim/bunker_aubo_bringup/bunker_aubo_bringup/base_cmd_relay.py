import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class BaseCmdRelay(Node):
    def __init__(self):
        super().__init__('base_cmd_relay')
        self.max_lin = self.declare_parameter('max_linear_velocity', 1.0).value
        self.max_ang = self.declare_parameter('max_angular_velocity', 1.2).value
        self.frame_id = self.declare_parameter('frame_id', 'base_footprint').value
        out_topic = self.declare_parameter('output_topic', '/diff_drive_controller/cmd_vel').value

        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(TwistStamped, out_topic, qos)
        self.create_subscription(Twist, '/cmd_vel', self.on_twist, qos)
        self.create_subscription(TwistStamped, '/cmd_vel_stamped', self.on_stamped, qos)
        self.get_logger().info(f'Relaying /cmd_vel -> {out_topic} '
                               f'(|v|<={self.max_lin} m/s, |w|<={self.max_ang} rad/s)')

    def _publish(self, twist: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist.linear.x = clamp(twist.linear.x, -self.max_lin, self.max_lin)
        out.twist.angular.z = clamp(twist.angular.z, -self.max_ang, self.max_ang)
        self.pub.publish(out)

    def on_twist(self, msg: Twist):
        self._publish(msg)

    def on_stamped(self, msg: TwistStamped):
        self._publish(msg.twist)


def main(args=None):
    rclpy.init(args=args)
    node = BaseCmdRelay()
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
