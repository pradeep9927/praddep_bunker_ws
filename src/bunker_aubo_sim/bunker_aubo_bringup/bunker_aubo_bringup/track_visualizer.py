"""Moving track treads for RViz.

The BUNKER body mesh (tracks included) is one rigid visual, so the belts look frozen while the
road wheels inside them turn. This node draws tread bars (grousers) along each track's outline
and moves them with the measured wheel rotation from /joint_states, exactly as a real belt moves:
belt travel = wheel angle x wheel radius. Forward driving moves the lower run backwards and the
upper run forwards; turning on the spot moves the two tracks in opposite directions.

    /joint_states --> track_visualizer --> /track_treads (visualization_msgs/MarkerArray)

Add a MarkerArray display on /track_treads in RViz (the bundled RViz configs already have it).
"""
import math
from typing import List, Sequence, Tuple

# Track outline in base_footprint (x forward, z up), just outside the BUNKER mesh, ordered in the
# direction the belt runs for POSITIVE wheel rotation (forward driving): along the bottom from
# front to rear, up the rear slope, along the top from rear to front, down the front slope.
TRACK_PATH = [(0.33, 0.006), (-0.33, 0.006), (-0.43, 0.04), (-0.52, 0.11), (-0.53, 0.20),
              (-0.47, 0.29), (-0.34, 0.375), (-0.10, 0.402), (0.20, 0.402), (0.36, 0.385),
              (0.47, 0.31), (0.515, 0.21), (0.46, 0.13), (0.40, 0.06)]


class TrackPath:
    def __init__(self, pts: Sequence[Tuple[float, float]] = TRACK_PATH):
        self.pts = list(pts)
        self.seg = []
        total = 0.0
        for i, a in enumerate(self.pts):
            b = self.pts[(i + 1) % len(self.pts)]
            ln = math.hypot(b[0] - a[0], b[1] - a[1])
            self.seg.append((total, ln, a, b))
            total += ln
        self.length = total

    def at(self, s: float) -> Tuple[float, float, float]:
        """(x, z, pitch) of the point at arc length s (wraps around)."""
        s %= self.length
        for start, ln, a, b in self.seg:
            if s <= start + ln or (start, ln, a, b) == self.seg[-1]:
                t = 0.0 if ln == 0 else (s - start) / ln
                x = a[0] + t * (b[0] - a[0])
                z = a[1] + t * (b[1] - a[1])
                # rotation about y that turns the box x-axis into the travel direction
                pitch = -math.atan2(b[1] - a[1], b[0] - a[0])
                return x, z, pitch
        raise RuntimeError('unreachable')

    def treads(self, travel: float, spacing: float) -> List[Tuple[float, float, float]]:
        n = max(1, int(round(self.length / spacing)))
        step = self.length / n
        return [self.at(travel + k * step) for k in range(n)]


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from visualization_msgs.msg import Marker, MarkerArray

    class TrackVisualizer(Node):
        def __init__(self):
            super().__init__('track_visualizer')
            self.frame = self.declare_parameter('frame_id', 'base_footprint').value
            self.radius = float(self.declare_parameter('wheel_radius', 0.075).value)
            self.spacing = float(self.declare_parameter('tread_spacing', 0.06).value)
            self.track_y = float(self.declare_parameter('track_y', 0.32).value)
            self.width = float(self.declare_parameter('track_width', 0.13).value)
            self.left = self.declare_parameter('left_joint', 'left_wheel_1_joint').value
            self.right = self.declare_parameter('right_joint', 'right_wheel_1_joint').value
            self.path = TrackPath()
            self.pub = self.create_publisher(MarkerArray, '/track_treads', 10)
            self.create_subscription(JointState, '/joint_states', self.on_joints, 20)
            self.last_pub = None

        def on_joints(self, msg: JointState):
            pos = dict(zip(msg.name, msg.position))
            if self.left not in pos or self.right not in pos:
                return
            now = self.get_clock().now()
            if self.last_pub is not None and (now - self.last_pub).nanoseconds < 40_000_000:
                return                                   # ~25 Hz is plenty for the display
            self.last_pub = now
            out = MarkerArray()
            mid = 0
            for side, joint, y in (('left', self.left, self.track_y), ('right', self.right, -self.track_y)):
                travel = pos[joint] * self.radius
                for x, z, pitch in self.path.treads(travel, self.spacing):
                    m = Marker()
                    m.header.frame_id = self.frame
                    m.header.stamp = msg.header.stamp
                    m.ns = f'{side}_track'
                    m.id = mid
                    mid += 1
                    m.type = Marker.CUBE
                    m.action = Marker.ADD
                    m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
                    m.pose.orientation.y = math.sin(pitch / 2.0)
                    m.pose.orientation.w = math.cos(pitch / 2.0)
                    m.scale.x, m.scale.y, m.scale.z = 0.022, self.width, 0.014
                    m.color.r = m.color.g = m.color.b = 0.12
                    m.color.a = 1.0
                    out.markers.append(m)
            self.pub.publish(out)

    rclpy.init(args=args)
    node = TrackVisualizer()
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
