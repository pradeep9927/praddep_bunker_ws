import math
import os
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from .map_io import cloud_fields_to_arrays, save_occupancy_pgm, save_ply


class ExportMaps(Node):
    def __init__(self):
        super().__init__('export_maps')
        self.output_dir = os.path.expanduser(self.declare_parameter('output_dir', 'maps').value)
        self.name = self.declare_parameter('name', 'map').value
        self.timeout = self.declare_parameter('timeout', 20.0).value
        self.map_topic = self.declare_parameter('map_topic', '/map').value
        self.cloud_topic = self.declare_parameter('cloud_topic', '/rtabmap/cloud_map').value
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid = None
        self.cloud = None
        self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, latched)
        self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, latched)

    def on_map(self, msg):
        self.grid = msg

    def on_cloud(self, msg):
        self.cloud = msg

    def run(self):
        end = time.monotonic() + self.timeout
        while rclpy.ok() and time.monotonic() < end and (self.grid is None or self.cloud is None):
            rclpy.spin_once(self, timeout_sec=0.2)
        os.makedirs(self.output_dir, exist_ok=True)
        base = os.path.join(self.output_dir, self.name)
        ok = True
        if self.grid is not None:
            m = self.grid
            grid = np.array(m.data, np.int16).reshape(m.info.height, m.info.width)
            q = m.info.origin.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            pgm, yml = save_occupancy_pgm(base, grid, m.info.resolution,
                                          (m.info.origin.position.x, m.info.origin.position.y, yaw))
            self.get_logger().info(f'2D map {m.info.width}x{m.info.height} @ {m.info.resolution} m -> {yml}')
        else:
            ok = False
            self.get_logger().error(f'no {self.map_topic} received within {self.timeout}s')
        if self.cloud is not None:
            xyz, rgb = cloud_fields_to_arrays(self.cloud)
            path = save_ply(base + '_cloud.ply', xyz, rgb)
            self.get_logger().info(f'3D map: {len(xyz)} points -> {path}')
        else:
            ok = False
            self.get_logger().error(f'no {self.cloud_topic} received within {self.timeout}s')
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = ExportMaps()
    ok = node.run()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
