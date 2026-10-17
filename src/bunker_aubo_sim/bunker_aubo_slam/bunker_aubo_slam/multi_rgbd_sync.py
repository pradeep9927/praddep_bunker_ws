import copy
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rtabmap_msgs.msg import RGBDImage, RGBDImages
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .sync_core import BLANK, SyncCore


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class MultiRgbdSync(Node):
    def __init__(self):
        super().__init__('multi_rgbd_sync')
        self.inputs = self.declare_parameter(
            'input_topics', ['/base_camera/rgbd_image/compressed', '/wrist_camera/rgbd_image/compressed']).value
        self.names = self.declare_parameter('camera_names', ['base_camera', 'wrist_camera']).value
        max_dt = self.declare_parameter('max_dt', 0.05).value
        timeout = self.declare_parameter('camera_timeout', 0.5).value
        pending = self.declare_parameter('pending_timeout', 0.3).value
        self.core = SyncCore(len(self.inputs), max_dt=max_dt, camera_timeout=timeout,
                             pending_timeout=pending)
        self.blank_cache = {}
        self.was_alive = [False] * len(self.inputs)
        self.enabled = [True] * len(self.inputs)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(RGBDImages, 'rgbd_images', qos)
        self.status_pub = self.create_publisher(String, '/slam/camera_status', 10)
        for i, topic in enumerate(self.inputs):
            self.create_subscription(RGBDImage, topic, lambda m, i=i: self.on_image(i, m), qos)
            self.create_service(SetBool, f'/slam/{self.names[i]}/enable',
                                lambda req, resp, i=i: self.on_enable(i, req, resp))
        self.create_timer(0.05, self.on_timer)
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info(f'Synchronising {self.inputs} -> {self.pub.topic_name} '
                               f'(max_dt={max_dt}s, camera_timeout={timeout}s)')

    def now(self) -> float:
        # ROS clock (sim time under Gazebo). With the wall clock, a simulation running slower than
        # real time made healthy cameras look "dead" and replaced them with blank frames.
        return self.get_clock().now().nanoseconds * 1e-9

    def on_enable(self, i, req, resp):
        self.enabled[i] = bool(req.data)
        resp.success = True
        resp.message = f'{self.names[i]} input {"enabled" if req.data else "DISABLED (simulated drop-out)"}'
        self.get_logger().warn(resp.message)
        return resp

    def on_image(self, i, msg: RGBDImage):
        if not self.enabled[i]:
            return
        self.emit(self.core.add(i, stamp_to_sec(msg.header.stamp), msg, self.now()))

    def on_timer(self):
        now = self.now()
        self.emit(self.core.flush(now))
        for i, name in enumerate(self.names):
            alive = self.core.alive(i, now)
            if alive != self.was_alive[i] and self.core.seen[i]:
                if alive:
                    self.get_logger().info(f'{name}: stream is back')
                else:
                    self.get_logger().warn(f'{name}: no data for {self.core.camera_timeout}s -> '
                                           'blank frames until it recovers (SLAM keeps running)')
            self.was_alive[i] = alive

    def emit(self, groups):
        for _, group in groups:
            ref = next(g for g in group if g is not BLANK)
            out = RGBDImages()
            out.header = ref.header
            for j, g in enumerate(group):
                out.rgbd_images.append(self.blank(j, ref.header.stamp) if g is BLANK else g)
            self.pub.publish(out)

    def blank(self, j, stamp) -> RGBDImage:
        template = self.core.buffers[j][-1].msg if self.core.buffers[j] else None
        if template is None:
            raise RuntimeError('blank requested for a camera that was never seen')
        info = template.rgb_camera_info
        key = (j, info.width, info.height)
        if key not in self.blank_cache:
            rgb = Image(height=info.height, width=info.width, encoding='rgb8', is_bigendian=0,
                        step=info.width * 3)
            rgb.data = bytes(info.width * info.height * 3)
            depth = Image(height=info.height, width=info.width, encoding='32FC1', is_bigendian=0,
                          step=info.width * 4)
            depth.data = bytes(info.width * info.height * 4)
            self.blank_cache[key] = (rgb, depth)
        rgb, depth = self.blank_cache[key]
        out = RGBDImage()
        out.header.stamp = stamp
        out.header.frame_id = template.header.frame_id
        out.rgb_camera_info = copy.deepcopy(template.rgb_camera_info)
        out.rgb_camera_info.header.stamp = stamp
        out.depth_camera_info = copy.deepcopy(template.depth_camera_info)
        out.depth_camera_info.header.stamp = stamp
        rgb.header.stamp = depth.header.stamp = stamp
        rgb.header.frame_id = template.rgb_camera_info.header.frame_id or template.header.frame_id
        depth.header.frame_id = template.depth_camera_info.header.frame_id or template.header.frame_id
        out.rgb = rgb
        out.depth = depth
        return out

    def publish_status(self):
        now = self.now()
        status = {name: {'alive': self.core.alive(i, now), 'enabled': self.enabled[i],
                         'blank_frames': self.core.blank_count[i]}
                  for i, name in enumerate(self.names)}
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = MultiRgbdSync()
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
