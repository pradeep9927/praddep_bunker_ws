import csv
import json
import math
import os
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .geometry import map_accuracy, se2, trajectory_errors, world_footprints

try:
    from rtabmap_msgs.msg import Info as RtabmapInfo
except ImportError:  # pragma: no cover
    RtabmapInfo = None


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def t_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class SlamEvaluator(Node):
    def __init__(self):
        super().__init__('slam_evaluator')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.world_file = self.declare_parameter('world_file', '').value
        self.output_dir = os.path.expanduser(self.declare_parameter('output_dir', '~/bunker_aubo_eval').value)
        self.sample_period = self.declare_parameter('sample_period', 0.2).value
        self.report_period = self.declare_parameter('report_period', 10.0).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.gt_queue = deque(maxlen=500)
        self.last_sample_t = -1e9
        self.rows = []
        self.loop_closures = []
        self.proximity = 0
        self.last_map = None
        self.world_T_map = None

        self.create_subscription(Odometry, '/ground_truth/odom', self.on_gt, 50)
        if RtabmapInfo is not None:
            self.create_subscription(RtabmapInfo, '/rtabmap/info', self.on_info, 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.pub = self.create_publisher(String, '/slam/evaluation', 10)
        self.create_service(Trigger, '/slam/save_evaluation', self.on_save)
        self.create_timer(0.1, self.process_queue)
        self.create_timer(self.report_period, self.report)
        self.shapes = None
        if self.world_file and os.path.isfile(self.world_file):
            self.shapes = world_footprints(self.world_file)
            self.get_logger().info(f'World geometry loaded: {len(self.shapes)} obstacle footprints')
        else:
            self.get_logger().warn('No world_file: map accuracy vs ground truth disabled')

    def on_gt(self, msg: Odometry):
        t = t_sec(msg.header.stamp)
        if t - self.last_sample_t < self.sample_period:
            return
        self.last_sample_t = t
        p = msg.pose.pose
        self.gt_queue.append((t, msg.header.stamp, p.position.x, p.position.y, yaw_of(p.orientation)))

    def lookup(self, parent, stamp):
        try:
            tf = self.tf_buffer.lookup_transform(parent, self.base_frame, Time.from_msg(stamp))
        except TransformException:
            return None
        tr = tf.transform
        return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)

    def process_queue(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        while self.gt_queue and now - self.gt_queue[0][0] > 0.3:
            t, stamp, gx, gy, gyaw = self.gt_queue.popleft()
            slam = self.lookup(self.map_frame, stamp)
            odom = self.lookup(self.odom_frame, stamp)
            if slam is None or odom is None:
                continue
            if self.world_T_map is None:
                self.world_T_map = se2(gx, gy, gyaw) @ np.linalg.inv(se2(*slam))
                self.world_T_odom = se2(gx, gy, gyaw) @ np.linalg.inv(se2(*odom))
            self.rows.append((t, gx, gy, gyaw, *slam, *odom))

    def on_info(self, msg):
        if msg.loop_closure_id > 0:
            self.loop_closures.append((t_sec(msg.header.stamp), msg.ref_id, msg.loop_closure_id))
            self.get_logger().info(f'LOOP CLOSURE #{len(self.loop_closures)}: node {msg.ref_id} '
                                   f'<-> node {msg.loop_closure_id}')
        if msg.proximity_detection_id > 0:
            self.proximity += 1

    def on_map(self, msg: OccupancyGrid):
        self.last_map = msg

    def compute(self):
        out = {'samples': len(self.rows), 'loop_closures': len(self.loop_closures),
               'proximity_detections': self.proximity}
        if len(self.rows) < 5:
            return out
        a = np.array(self.rows)
        gt = a[:, 1:3]
        slam_w = (self.world_T_map[:2, :2] @ a[:, 4:6].T).T + self.world_T_map[:2, 2]
        odom_w = (self.world_T_odom[:2, :2] @ a[:, 7:9].T).T + self.world_T_odom[:2, 2]
        out['slam_anchored'] = trajectory_errors(slam_w, gt, align=False)
        out['slam_aligned'] = trajectory_errors(a[:, 4:6], gt, align=True)
        out['odom_anchored'] = trajectory_errors(odom_w, gt, align=False)
        out['odom_aligned'] = trajectory_errors(a[:, 7:9], gt, align=True)
        yaw_err = np.degrees(np.abs(np.angle(np.exp(1j * (a[:, 6] + math.atan2(
            self.world_T_map[1, 0], self.world_T_map[0, 0]) - a[:, 3])))))
        out['slam_yaw_error_deg'] = {'mean': float(yaw_err.mean()), 'max': float(yaw_err.max()),
                                     'final': float(yaw_err[-1])}
        steps = np.linalg.norm(np.diff(slam_w, axis=0), axis=1)
        gsteps = np.linalg.norm(np.diff(gt, axis=0), axis=1)
        out['slam_max_step_minus_truth_m'] = float(np.max(np.abs(steps - gsteps))) if len(steps) else 0.0
        if self.last_map is not None and self.shapes is not None and self.world_T_map is not None:
            m = self.last_map
            grid = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
            o = m.info.origin
            if abs(yaw_of(o.orientation)) < 1e-6:
                out['map_2d'] = map_accuracy(grid, m.info.resolution, (o.position.x, o.position.y),
                                             self.world_T_map, self.shapes)
        return out

    def report(self):
        res = self.compute()
        self.pub.publish(String(data=json.dumps(res)))
        if 'slam_anchored' in res:
            s, o = res['slam_anchored'], res['odom_anchored']
            self.get_logger().info(
                f'[eval] path {s["path_length"]:.1f} m | SLAM ATE rmse {s["ate_rmse"]:.3f} m '
                f'(final {s["final_error"]:.3f} m) | odometry-only ATE rmse {o["ate_rmse"]:.3f} m '
                f'(final {o["final_error"]:.3f} m) | loop closures {res["loop_closures"]}')

    def save(self):
        os.makedirs(self.output_dir, exist_ok=True)
        res = self.compute()
        with open(os.path.join(self.output_dir, 'trajectory.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'gt_x', 'gt_y', 'gt_yaw', 'slam_x', 'slam_y', 'slam_yaw',
                        'odom_x', 'odom_y', 'odom_yaw'])
            w.writerows(self.rows)
        with open(os.path.join(self.output_dir, 'loop_closures.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'ref_id', 'loop_closure_id'])
            w.writerows(self.loop_closures)
        with open(os.path.join(self.output_dir, 'evaluation.json'), 'w') as f:
            json.dump(res, f, indent=2)
        with open(os.path.join(self.output_dir, 'evaluation.md'), 'w') as f:
            f.write(self.markdown(res))
        self.plot()
        return res

    @staticmethod
    def markdown(res):
        lines = ['# SLAM evaluation vs Gazebo ground truth', '',
                 f'* samples: {res.get("samples", 0)} (5 Hz)',
                 f'* loop closures: {res.get("loop_closures", 0)}, '
                 f'proximity detections: {res.get("proximity_detections", 0)}', '']
        if 'slam_anchored' in res:
            lines += ['| Estimate | ATE RMSE [m] | mean [m] | max [m] | final [m] | final [% of path] |',
                      '|---|---|---|---|---|---|']
            for key, label in (('slam_anchored', 'SLAM (anchored at start)'),
                               ('slam_aligned', 'SLAM (best rigid fit)'),
                               ('odom_anchored', 'EKF odometry only (anchored)'),
                               ('odom_aligned', 'EKF odometry only (best fit)')):
                e = res[key]
                lines.append(f'| {label} | {e["ate_rmse"]:.3f} | {e["ate_mean"]:.3f} | {e["ate_max"]:.3f} '
                             f'| {e["final_error"]:.3f} | {e.get("final_error_percent", float("nan")):.2f} |')
            lines += ['', f'Path length: {res["slam_anchored"]["path_length"]:.1f} m. '
                      f'SLAM heading error mean {res["slam_yaw_error_deg"]["mean"]:.2f} deg, '
                      f'max {res["slam_yaw_error_deg"]["max"]:.2f} deg.', '']
        if 'map_2d' in res:
            m = res['map_2d']
            lines += ['## 2D occupancy map vs world geometry', '',
                      f'* occupied cells: {m["occupied_cells"]}',
                      f'* distance to true obstacle: mean {m["mean_error_m"]:.3f} m, '
                      f'median {m["median_error_m"]:.3f} m, 95th pct {m["p95_error_m"]:.3f} m',
                      f'* cells within 5 cm: {100 * m["within_5cm"]:.1f} %, within 10 cm: '
                      f'{100 * m["within_10cm"]:.1f} %',
                      f'* observed walls covered by the map (15 cm tolerance): {100 * m["wall_coverage"]:.1f} %', '']
        return '\n'.join(lines) + '\n'

    def plot(self):
        if len(self.rows) < 5:
            return
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            return
        a = np.array(self.rows)
        slam_w = (self.world_T_map[:2, :2] @ a[:, 4:6].T).T + self.world_T_map[:2, 2]
        odom_w = (self.world_T_odom[:2, :2] @ a[:, 7:9].T).T + self.world_T_odom[:2, 2]
        fig, ax = plt.subplots(figsize=(9, 7))
        ax.plot(a[:, 1], a[:, 2], 'k-', lw=2, label='ground truth')
        ax.plot(slam_w[:, 0], slam_w[:, 1], 'r-', lw=1.2, label='SLAM (map frame)')
        ax.plot(odom_w[:, 0], odom_w[:, 1], 'b--', lw=1, label='EKF odometry only')
        for t, _, _ in self.loop_closures:
            i = int(np.argmin(np.abs(a[:, 0] - t)))
            ax.plot(slam_w[i, 0], slam_w[i, 1], 'g*', ms=12)
        ax.plot([], [], 'g*', ms=12, label='loop closure')
        ax.set_aspect('equal')
        ax.grid(alpha=0.3)
        ax.legend()
        ax.set_xlabel('world x [m]')
        ax.set_ylabel('world y [m]')
        ax.set_title('Trajectory vs Gazebo ground truth')
        fig.tight_layout()
        fig.savefig(os.path.join(self.output_dir, 'trajectory.png'), dpi=110)
        plt.close(fig)

    def on_save(self, _req, resp):
        try:
            res = self.save()
            resp.success = True
            resp.message = f'saved to {self.output_dir}: ' + json.dumps(
                {k: res[k] for k in ('samples', 'loop_closures') if k in res})
        except Exception as exc:  # noqa: BLE001
            resp.success = False
            resp.message = str(exc)
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = SlamEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.rows:
                node.save()
                print(f'[slam_evaluator] evaluation written to {node.output_dir}')
        except Exception as exc:  # noqa: BLE001
            print(f'[slam_evaluator] could not save evaluation: {exc}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
