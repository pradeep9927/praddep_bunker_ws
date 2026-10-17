import math

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets

from .arm_model import ArmModel
from .msg_utils import height_colors
from .topic_monitor import NO_DATA, OK

try:
    import pyqtgraph.opengl as gl
    HAVE_GL = True
except Exception:  # noqa: BLE001
    gl = None
    HAVE_GL = False

STATE_COLOURS = {OK: '#2e7d32', 'STALE': '#ef6c00', NO_DATA: '#9e9e9e'}
ESTOP_IDLE_STYLE = 'QPushButton{background:#c62828;color:white;font-weight:bold;font-size:18px;}'
ESTOP_ACTIVE_STYLE = 'QPushButton{background:#ffeb3b;color:#b71c1c;font-weight:bold;font-size:18px;}'


class ImageView(QtWidgets.QLabel):
    def __init__(self, title):
        super().__init__()
        self.title = title
        self.setMinimumSize(320, 240)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setStyleSheet('background-color: #111; color: #bbb; font-size: 16px;')
        self._last_seq = -1
        self.show_placeholder('waiting for data ...')

    def show_placeholder(self, text):
        self.setPixmap(QtGui.QPixmap())
        self.setText(f'{self.title}\n\n{text}')

    def update_frame(self, rgb, seq, stale):
        if rgb is None:
            self.show_placeholder('NO SIGNAL')
            return
        if stale:
            self.show_placeholder('SIGNAL LOST (stale)')
            self._last_seq = -1
            return
        if seq == self._last_seq:
            return
        self._last_seq = seq
        h, w, _ = rgb.shape
        buf = rgb.tobytes()
        img = QtGui.QImage(buf, w, h, 3 * w, QtGui.QImage.Format_RGB888).copy()  # own the pixels
        pix = QtGui.QPixmap.fromImage(img).scaled(self.width(), self.height(), QtCore.Qt.KeepAspectRatio,
                                                  QtCore.Qt.FastTransformation)
        self.setPixmap(pix)


def _share_dir(pkg):
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(pkg)


def _qmatrix(tf):
    return QtGui.QMatrix4x4(*[float(v) for v in np.asarray(tf).reshape(-1)])


class LidarView(QtWidgets.QWidget):
    """3D view of the LiDAR cloud with the robot model, both in base_footprint (floor at z = 0)."""

    def __init__(self, view_cfg=None):
        super().__init__()
        view_cfg = view_cfg or {}
        self.zmin = float(view_cfg.get('color_min_z', 0.0))
        self.zmax = float(view_cfg.get('color_max_z', 2.5))
        self.scene = None            # RobotScene, built once /robot_description arrives
        self.robot_items = []
        self._robot_failed = False
        self._cloud_tf = None
        self._cloud_tf_frame = None
        self._last_seq = -1
        self._last_q = None

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        if HAVE_GL:
            self.view = gl.GLViewWidget()
            self.view.setCameraPosition(pos=QtGui.QVector3D(0.0, 0.0, 0.6),
                                        distance=7.5, elevation=28, azimuth=-135)
            self.view.setBackgroundColor((20, 20, 25))
            grid = gl.GLGridItem()
            grid.setSize(20, 20)
            grid.setSpacing(1, 1)
            self.view.addItem(grid)
            axis = gl.GLAxisItem()
            axis.setSize(0.5, 0.5, 0.5)
            self.view.addItem(axis)
            self.scatter = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), size=2.0, pxMode=True)
            self.view.addItem(self.scatter)
            lay.addWidget(self.view)
        else:
            self.view = pg.PlotWidget(title='LiDAR (top view, OpenGL unavailable)')
            self.view.setAspectLocked(True)
            self.robot_outline = pg.ScatterPlotItem(size=2, pen=None, brush=pg.mkBrush(200, 200, 200, 120))
            self.view.addItem(self.robot_outline)
            self.scatter = pg.ScatterPlotItem(size=2, pen=None)
            self.view.addItem(self.scatter)
            lay.addWidget(self.view)

        row = QtWidgets.QHBoxLayout()
        self.info = QtWidgets.QLabel('LiDAR: waiting for /lidar/points ...')
        self.show_robot = QtWidgets.QCheckBox('show robot')
        self.show_robot.setChecked(bool(view_cfg.get('show_robot', True)))
        self.show_robot.setFocusPolicy(QtCore.Qt.NoFocus)
        self.show_robot.toggled.connect(self._set_robot_visible)
        row.addWidget(self.info, stretch=1)
        row.addWidget(self.show_robot)
        lay.addLayout(row)

    # -------------------------------------------------------------------- robot model
    def set_robot(self, urdf_text):
        """Build the robot model from the URDF. Returns True when the model is shown."""
        if self.scene is not None or self._robot_failed or not urdf_text:
            return self.scene is not None
        try:
            from .robot_view import RobotScene
            scene = RobotScene(urdf_text, _share_dir)
        except Exception as exc:  # noqa: BLE001 - the GUI must keep running without the model
            self._robot_failed = True
            self.info.setText(f'robot model not available: {exc}')
            return False
        self.scene = scene
        if HAVE_GL:
            for part in scene.parts:
                item = gl.GLMeshItem(meshdata=gl.MeshData(vertexes=part.triangles.astype(np.float32)),
                                     color=part.rgba, shader='shaded', smooth=False,
                                     drawEdges=False, glOptions='opaque')
                item.setVisible(self.show_robot.isChecked())
                self.view.addItem(item)
                self.robot_items.append(item)
        self.update_robot({})
        return True

    def _set_robot_visible(self, on):
        for item in self.robot_items:
            item.setVisible(on)
        if not HAVE_GL and self.scene is not None:
            self.robot_outline.setVisible(on)

    def update_robot(self, positions):
        if self.scene is None:
            return
        q = tuple(round(v, 3) for _, v in sorted(positions.items()))
        if q == self._last_q:
            return
        self._last_q = q
        if HAVE_GL:
            for item, tf in zip(self.robot_items, self.scene.part_transforms(positions)):
                item.setTransform(_qmatrix(tf))
        else:
            xy = self.scene.outline_xy(positions)
            self.robot_outline.setData(x=xy[:, 0], y=xy[:, 1])

    # -------------------------------------------------------------------- point cloud
    def _cloud_to_base(self, frame):
        if self.scene is None or not frame:
            return None
        if frame != self._cloud_tf_frame:
            self._cloud_tf_frame = frame
            self._cloud_tf = self.scene.frame_to_root(frame)
        return self._cloud_tf

    def update_cloud(self, xyz, frame, seq, stale):
        if xyz is None:
            return
        if stale:
            self.info.setText('LiDAR: SIGNAL LOST (showing last cloud)')
            return
        if seq == self._last_seq:
            return
        self._last_seq = seq
        tf = self._cloud_to_base(frame)
        if tf is not None:
            from .robot_view import transform_points
            xyz = transform_points(xyz, tf)
            where = self.scene.root_frame
        else:
            where = frame or 'sensor frame'
        cols = height_colors(xyz[:, 2], self.zmin, self.zmax)
        if HAVE_GL:
            self.scatter.setData(pos=xyz, color=cols, size=2.0)
        else:
            brushes = [pg.mkBrush(int(r * 255), int(g * 255), int(b * 255)) for r, g, b, _ in cols[::4]]
            self.scatter.setData(x=xyz[::4, 0], y=xyz[::4, 1], brush=brushes)
        self.info.setText(f'LiDAR: {len(xyz)} points in {where} (colour = height)')


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, ros, cfg):
        super().__init__()
        self.ros = ros
        self.cfg = cfg
        self.setWindowTitle(cfg['gui']['window_title'])
        self.resize(1600, 980)

        arm_cfg = cfg['arm']
        self.arm = ArmModel(arm_cfg['joints'], arm_cfg['fallback_limits'], arm_cfg['named_poses'])
        self.estop = False
        self.drive_keys = set()      # active keyboard directions
        self.drive_buttons = set()   # active on-screen directions
        self.zero_repeats = 0

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)

        left = QtWidgets.QVBoxLayout()
        left.addWidget(self._build_base_panel())
        left.addWidget(self._build_arm_panel())
        left.addWidget(self._build_gripper_panel())
        left.addStretch(1)
        left_w = QtWidgets.QWidget()
        left_w.setLayout(left)
        left_w.setFixedWidth(470)
        root.addWidget(left_w)

        centre = QtWidgets.QVBoxLayout()
        centre.addLayout(self._build_camera_row(), stretch=5)
        centre.addWidget(self._build_lidar_and_status(), stretch=6)
        root.addLayout(centre, stretch=1)

        self._timer(1000.0 / cfg['gui']['image_refresh_hz'], self.refresh_images)
        self._timer(1000.0 / cfg['gui']['lidar_refresh_hz'], self.refresh_lidar)
        self._timer(1000.0 / cfg['gui']['status_refresh_hz'], self.refresh_status)
        self._timer(1000.0 / cfg['base']['publish_rate'], self.drive_tick)
        self._timer(100.0, self.refresh_joints)

        QtWidgets.QApplication.instance().installEventFilter(self)
        self.statusBar().showMessage('Ready. Keyboard: W/A/S/D or arrows, Space = stop, Esc = E-STOP')

    def _timer(self, period_ms, fn):
        t = QtCore.QTimer(self)
        t.timeout.connect(fn)
        t.start(int(period_ms))
        return t

    def _build_base_panel(self):
        box = QtWidgets.QGroupBox('Mobile base (BUNKER)')
        lay = QtWidgets.QGridLayout(box)
        b = self.cfg['base']

        self.lin_slider, self.lin_label = self._speed_slider(b['default_linear'], b['max_linear'], 'm/s')
        self.ang_slider, self.ang_label = self._speed_slider(b['default_angular'], b['max_angular'], 'rad/s')
        lay.addWidget(QtWidgets.QLabel('Linear speed'), 0, 0)
        lay.addWidget(self.lin_slider, 0, 1)
        lay.addWidget(self.lin_label, 0, 2)
        lay.addWidget(QtWidgets.QLabel('Turn rate'), 1, 0)
        lay.addWidget(self.ang_slider, 1, 1)
        lay.addWidget(self.ang_label, 1, 2)

        pad = QtWidgets.QGridLayout()
        buttons = {
            'fl': ('↖', 0, 0, (1, 1)), 'f': ('▲ Fwd', 0, 1, (1, 0)), 'fr': ('↗', 0, 2, (1, -1)),
            'l': ('◀ Left', 1, 0, (0, 1)), 'r': ('Right ▶', 1, 2, (0, -1)),
            'bl': ('↙', 2, 0, (-1, -1)), 'b': ('▼ Back', 2, 1, (-1, 0)), 'br': ('↘', 2, 2, (-1, 1)),
        }
        self.direction_of = {}
        for key, (text, r, c, direction) in buttons.items():
            btn = QtWidgets.QPushButton(text)
            btn.setMinimumHeight(38)
            btn.setFocusPolicy(QtCore.Qt.NoFocus)
            btn.pressed.connect(lambda k=key: self.drive_buttons.add(k))
            btn.released.connect(lambda k=key: self._release_button(k))
            self.direction_of[key] = direction
            pad.addWidget(btn, r, c)
        stop = QtWidgets.QPushButton('■ Stop')
        stop.setMinimumHeight(38)
        stop.setFocusPolicy(QtCore.Qt.NoFocus)
        stop.clicked.connect(self.stop_base)
        pad.addWidget(stop, 1, 1)
        lay.addLayout(pad, 2, 0, 1, 3)

        self.estop_btn = QtWidgets.QPushButton('EMERGENCY STOP')
        self.estop_btn.setMinimumHeight(56)
        self.estop_btn.setFocusPolicy(QtCore.Qt.NoFocus)
        self.estop_btn.setStyleSheet(ESTOP_IDLE_STYLE)
        self.estop_btn.clicked.connect(self.toggle_estop)
        lay.addWidget(self.estop_btn, 3, 0, 1, 3)

        self.kbd_check = QtWidgets.QCheckBox('Keyboard drive (W/A/S/D, arrows, Space, Esc)')
        self.kbd_check.setChecked(True)
        lay.addWidget(self.kbd_check, 4, 0, 1, 3)
        self.base_state = QtWidgets.QLabel('cmd: 0.00 m/s, 0.00 rad/s | measured: -')
        lay.addWidget(self.base_state, 5, 0, 1, 3)
        return box

    def _speed_slider(self, default, maximum, unit):
        s = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        s.setRange(0, int(maximum * 100))
        s.setValue(int(default * 100))
        s.setFocusPolicy(QtCore.Qt.NoFocus)
        lab = QtWidgets.QLabel(f'{default:.2f} {unit}')
        s.valueChanged.connect(lambda v: lab.setText(f'{v / 100:.2f} {unit}'))
        return s, lab

    def _release_button(self, key):
        self.drive_buttons.discard(key)
        self.zero_repeats = 5

    def current_command(self):
        lin = self.lin_slider.value() / 100.0
        ang = self.ang_slider.value() / 100.0
        fwd = turn = 0
        for k in self.drive_buttons:
            f, t = self.direction_of[k]
            fwd += f
            turn += t
        for k in self.drive_keys:
            f, t = {'f': (1, 0), 'b': (-1, 0), 'l': (0, 1), 'r': (0, -1)}[k]
            fwd += f
            turn += t
        fwd = max(-1, min(1, fwd))
        turn = max(-1, min(1, turn))
        return fwd * lin, turn * ang

    def drive_tick(self):
        if self.estop:
            self.ros.publish_twist(0.0, 0.0)
            v, w = 0.0, 0.0
        else:
            v, w = self.current_command()
            if v != 0.0 or w != 0.0:
                self.ros.publish_twist(v, w)
                self.zero_repeats = 5
            elif self.zero_repeats > 0:
                self.ros.publish_twist(0.0, 0.0)
                self.zero_repeats -= 1
        mv, mw = self.ros.odom_twist()
        self.base_state.setText(f'cmd: {v:+.2f} m/s, {w:+.2f} rad/s | '
                                f'measured (EKF): {mv:+.2f} m/s, {mw:+.2f} rad/s')

    def stop_base(self):
        self.drive_buttons.clear()
        self.drive_keys.clear()
        self.zero_repeats = 10
        self.ros.publish_twist(0.0, 0.0)

    def toggle_estop(self):
        self.estop = not self.estop
        if self.estop:
            self.stop_base()
            self.ros.cancel_arm()
            self.ros.cancel_gripper()
            self.estop_btn.setText('E-STOP ACTIVE: click to release')
            self.estop_btn.setStyleSheet(ESTOP_ACTIVE_STYLE)
            self.statusBar().showMessage('E-STOP: base held at zero velocity, arm/gripper goals cancelled')
        else:
            self.estop_btn.setText('EMERGENCY STOP')
            self.estop_btn.setStyleSheet(ESTOP_IDLE_STYLE)
            self.statusBar().showMessage('E-STOP released')

    KEYMAP = {QtCore.Qt.Key_W: 'f', QtCore.Qt.Key_Up: 'f', QtCore.Qt.Key_S: 'b', QtCore.Qt.Key_Down: 'b',
              QtCore.Qt.Key_A: 'l', QtCore.Qt.Key_Left: 'l', QtCore.Qt.Key_D: 'r', QtCore.Qt.Key_Right: 'r'}

    def eventFilter(self, obj, event):
        if event.type() in (QtCore.QEvent.KeyPress, QtCore.QEvent.KeyRelease) and self.isActiveWindow():
            if isinstance(QtWidgets.QApplication.focusWidget(), (QtWidgets.QLineEdit, QtWidgets.QAbstractSpinBox)):
                return False
            key = event.key()
            if event.type() == QtCore.QEvent.KeyPress and key == QtCore.Qt.Key_Escape:
                if not self.estop:
                    self.toggle_estop()
                return True
            if not self.kbd_check.isChecked():
                return False
            if key == QtCore.Qt.Key_Space and event.type() == QtCore.QEvent.KeyPress:
                self.stop_base()
                return True
            if key in self.KEYMAP:
                if event.isAutoRepeat():
                    return True
                if event.type() == QtCore.QEvent.KeyPress:
                    self.drive_keys.add(self.KEYMAP[key])
                else:
                    self.drive_keys.discard(self.KEYMAP[key])
                    self.zero_repeats = 5
                return True
        return False

    def changeEvent(self, event):
        if event.type() == QtCore.QEvent.ActivationChange and not self.isActiveWindow():
            if self.drive_keys:
                self.drive_keys.clear()
                self.zero_repeats = 5
        super().changeEvent(event)

    def _build_arm_panel(self):
        box = QtWidgets.QGroupBox('Arm (AUBO-i5): joint control')
        lay = QtWidgets.QGridLayout(box)
        self.joint_sliders, self.joint_spins, self.joint_actual = [], [], []
        for i, j in enumerate(self.arm.joints):
            lim = self.arm.limits[j]
            lay.addWidget(QtWidgets.QLabel(j.replace('_joint', '')), i, 0)
            s = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            s.setRange(int(math.degrees(lim.lower) * 10), int(math.degrees(lim.upper) * 10))
            s.setFocusPolicy(QtCore.Qt.ClickFocus)
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(math.degrees(lim.lower), math.degrees(lim.upper))
            sp.setDecimals(1)
            sp.setSuffix('°')
            sp.setKeyboardTracking(False)
            s.valueChanged.connect(lambda v, sp=sp: sp.setValue(v / 10.0))
            sp.valueChanged.connect(lambda v, s=s: s.setValue(int(round(v * 10))))
            s.sliderReleased.connect(self._maybe_live_send)
            sp.valueChanged.connect(self._update_target_check)
            act = QtWidgets.QLabel('-')
            act.setMinimumWidth(60)
            lay.addWidget(s, i, 1)
            lay.addWidget(sp, i, 2)
            lay.addWidget(act, i, 3)
            self.joint_sliders.append(s)
            self.joint_spins.append(sp)
            self.joint_actual.append(act)
        row = len(self.arm.joints)
        lay.addWidget(QtWidgets.QLabel('<i>target</i>'), row, 2)
        lay.addWidget(QtWidgets.QLabel('<i>actual</i>'), row, 3)

        btns = QtWidgets.QHBoxLayout()
        send = QtWidgets.QPushButton('Move to target')
        send.clicked.connect(self.send_arm_target)
        sync = QtWidgets.QPushButton('Target ← actual')
        sync.clicked.connect(self.sync_sliders)
        stop = QtWidgets.QPushButton('Stop arm')
        stop.clicked.connect(self.ros.cancel_arm)
        for b in (send, sync, stop):
            btns.addWidget(b)
        lay.addLayout(btns, row + 1, 0, 1, 4)

        self.live_check = QtWidgets.QCheckBox('Live: move when a slider is released')
        lay.addWidget(self.live_check, row + 2, 0, 1, 4)

        poses = QtWidgets.QHBoxLayout()
        for name in self.arm.named_poses:
            b = QtWidgets.QPushButton(name.capitalize())
            b.clicked.connect(lambda _, n=name: self.go_named(n))
            poses.addWidget(b)
        lay.addWidget(QtWidgets.QLabel('Named poses:'), row + 3, 0, 1, 4)
        lay.addLayout(poses, row + 4, 0, 1, 4)

        self.arm_state = QtWidgets.QLabel('arm: waiting for /joint_states')
        self.arm_state.setWordWrap(True)
        lay.addWidget(self.arm_state, row + 5, 0, 1, 4)
        self.collision_label = QtWidgets.QLabel('target: collision check not available yet')
        self.collision_label.setWordWrap(True)
        lay.addWidget(self.collision_label, row + 6, 0, 1, 4)
        self._sliders_synced = False
        return box

    def _slider_targets(self):
        return [math.radians(sp.value()) for sp in self.joint_spins]

    def _apply_limits_to_widgets(self):
        for j, s, sp in zip(self.arm.joints, self.joint_sliders, self.joint_spins):
            lim = self.arm.limits[j]
            s.setRange(int(math.degrees(lim.lower) * 10), int(math.degrees(lim.upper) * 10))
            sp.setRange(math.degrees(lim.lower), math.degrees(lim.upper))

    def set_sliders(self, q):
        for v, sp in zip(q, self.joint_spins):
            sp.setValue(math.degrees(v))

    def sync_sliders(self):
        q = self.ros.joint_positions(self.arm.joints)
        if q is not None:
            self.set_sliders(self.arm.clamp(q))

    def _maybe_live_send(self):
        if self.live_check.isChecked():
            self.send_arm_target()

    def send_arm_target(self):
        self._send_arm(self._slider_targets())

    def go_named(self, name):
        q = self.arm.named_poses[name]
        self.set_sliders(q)
        self._send_arm(q)

    def _gripper_q(self):
        return self.ros.joint_positions(self.cfg['gripper']['joints'])

    def _update_target_check(self, *_):
        if not hasattr(self, 'collision_label'):
            return
        text = self.arm.target_status(self.arm.clamp(self._slider_targets()), self._gripper_q())
        self.collision_label.setText(text)
        self.collision_label.setStyleSheet('color:#c62828;font-weight:bold;' if 'COLLIDES' in text else
                                           'color:#2e7d32;')

    def _send_arm(self, target):
        if self.estop:
            self.statusBar().showMessage('E-STOP active: arm motion blocked')
            return
        clamped = self.arm.clamp(target)
        if clamped != list(target):
            self.statusBar().showMessage('Target clamped to joint limits')
        current = self.ros.joint_positions(self.arm.joints)
        ok, msg = self.arm.check_motion(current, clamped, self._gripper_q())
        if not ok:
            self.statusBar().showMessage(f'Arm motion REFUSED: {msg}')
            self.collision_label.setText(f'last command refused: {msg}')
            self.collision_label.setStyleSheet('color:#c62828;font-weight:bold;')
            return
        a = self.cfg['arm']
        dur = self.arm.move_duration(current, clamped, a['velocity_scale'], a['min_move_time'])
        self.ros.send_arm_goal(self.arm.joints, clamped, dur)

    def refresh_joints(self):
        if self.arm.source.startswith('gui.yaml'):
            urdf = self.ros.robot_description()
            if urdf and self.arm.update_from_urdf(urdf):
                self._apply_limits_to_widgets()
                self._update_target_check()
        q = self.ros.joint_positions(self.arm.joints)
        if q is not None:
            for v, lab in zip(q, self.joint_actual):
                lab.setText(f'{math.degrees(v):7.1f}°')
            if not self._sliders_synced:
                self.set_sliders(self.arm.clamp(q))
                self._sliders_synced = True
        ready = 'ready' if self.ros.arm_server_ready() else 'NOT AVAILABLE'
        self.arm_state.setText(f'controller: {ready} | status: {self.ros.arm_status} | limits from {self.arm.source}')
        g = self.ros.joint_positions(self.cfg['gripper']['joints'])
        if g is not None:
            self.grip_actual.setText(f'opening: {sum(g) * 1000:.1f} mm (measured)')
            if not self._grip_slider_synced:      # start the slider at the real opening
                self._set_grip_slider(sum(g))
                self._grip_slider_synced = True
        gready = 'ready' if self.ros.gripper_server_ready() else 'NOT AVAILABLE'
        self.grip_state.setText(f'gripper action: {gready} | {self.ros.gripper_status}')

    def _build_gripper_panel(self):
        box = QtWidgets.QGroupBox('Gripper')
        lay = QtWidgets.QGridLayout(box)
        gcfg = self.cfg['gripper']
        maxmm = int(round(gcfg['max_opening'] * 1000))
        open_b = QtWidgets.QPushButton('Open')
        open_b.clicked.connect(lambda: self._grip(gcfg['max_opening']))
        close_b = QtWidgets.QPushButton('Close')
        close_b.clicked.connect(lambda: self._grip(0.0))
        self.grip_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.grip_slider.setRange(0, maxmm)
        self.grip_slider.setValue(maxmm)
        self.grip_slider.setSingleStep(1)
        self.grip_slider.setPageStep(10)
        self.grip_slider.setFocusPolicy(QtCore.Qt.ClickFocus)
        self.grip_target_lab = QtWidgets.QLabel(f'{maxmm} mm')
        set_b = QtWidgets.QPushButton('Set width')
        set_b.clicked.connect(lambda: self._grip(self.grip_slider.value() / 1000.0))
        self.grip_live = QtWidgets.QCheckBox('move with the slider')
        self.grip_live.setChecked(bool(gcfg.get('slider_moves_gripper', True)))
        self.grip_live.setFocusPolicy(QtCore.Qt.NoFocus)

        # the slider sends one command once it stops moving (drag, click on the groove, keys)
        self._grip_debounce = QtCore.QTimer(self)
        self._grip_debounce.setSingleShot(True)
        self._grip_debounce.setInterval(int(gcfg.get('slider_delay_ms', 250)))
        self._grip_debounce.timeout.connect(lambda: self._grip(self.grip_slider.value() / 1000.0))
        self.grip_slider.valueChanged.connect(self._grip_slider_moved)
        self.grip_slider.sliderReleased.connect(self._grip_slider_released)

        lay.addWidget(open_b, 0, 0)
        lay.addWidget(close_b, 0, 1)
        lay.addWidget(self.grip_slider, 1, 0)
        lay.addWidget(self.grip_target_lab, 1, 1)
        lay.addWidget(set_b, 1, 2)
        lay.addWidget(self.grip_live, 2, 0, 1, 3)
        self.grip_actual = QtWidgets.QLabel('opening: -')
        self.grip_state = QtWidgets.QLabel('gripper action: -')
        lay.addWidget(self.grip_actual, 3, 0, 1, 3)
        lay.addWidget(self.grip_state, 4, 0, 1, 3)
        self._grip_slider_synced = False
        return box

    def _set_grip_slider(self, gap):
        """Move the slider without sending a command."""
        mm = int(round(gap * 1000))
        self.grip_slider.blockSignals(True)
        self.grip_slider.setValue(mm)
        self.grip_slider.blockSignals(False)
        self.grip_target_lab.setText(f'{self.grip_slider.value()} mm')

    def _grip_slider_moved(self, value):
        self.grip_target_lab.setText(f'{value} mm')
        if self.grip_live.isChecked() and not self.grip_slider.isSliderDown():
            self._grip_debounce.start()          # click / keys: send after a short pause

    def _grip_slider_released(self):
        if self.grip_live.isChecked():
            self._grip_debounce.stop()
            self._grip(self.grip_slider.value() / 1000.0)

    def _grip(self, gap):
        self._grip_debounce.stop()
        if self.estop:
            self.statusBar().showMessage('E-STOP active: gripper motion blocked')
            return
        gap = min(max(gap, 0.0), self.cfg['gripper']['max_opening'])
        self._set_grip_slider(gap)               # Open / Close keep the slider in step
        self._grip_slider_synced = True
        if not self.ros.send_gripper(gap):
            self.statusBar().showMessage('gripper action server not available')

    def _build_camera_row(self):
        row = QtWidgets.QHBoxLayout()
        self.cam_views = {}
        self.depth_checks = {}
        for key, title in (('base', 'Base camera (mast D435)'), ('wrist', 'Wrist camera (D435)')):
            col = QtWidgets.QVBoxLayout()
            view = ImageView(title)
            chk = QtWidgets.QCheckBox(f'{title}: show depth')
            chk.setFocusPolicy(QtCore.Qt.NoFocus)
            col.addWidget(view, stretch=1)
            col.addWidget(chk)
            row.addLayout(col)
            self.cam_views[key] = view
            self.depth_checks[key] = chk
        return row

    def _build_lidar_and_status(self):
        split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.lidar = LidarView(self.cfg.get('lidar_view', {}))
        split.addWidget(self.lidar)

        right = QtWidgets.QWidget()
        rl = QtWidgets.QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        self.status_table = QtWidgets.QTableWidget(0, 4)
        self.status_table.setHorizontalHeaderLabels(['Topic', 'Status', 'Rate [Hz]', 'Age [s]'])
        self.status_table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        self.status_table.verticalHeader().setVisible(False)
        self.status_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.status_table.setFocusPolicy(QtCore.Qt.NoFocus)
        rl.addWidget(QtWidgets.QLabel('<b>Sensor / topic status</b>'))
        rl.addWidget(self.status_table)
        self.slam_label = QtWidgets.QLabel('SLAM: -')
        rl.addWidget(self.slam_label)
        split.addWidget(right)
        split.setSizes([700, 420])
        return split

    def _stale(self, topic):
        return self.ros.monitor.status(topic).state != OK

    def refresh_images(self):
        for key, view in self.cam_views.items():
            kind = 'depth' if self.depth_checks[key].isChecked() else 'color'
            topic = self.cfg['cameras'][key][kind]
            rgb, seq = self.ros.latest_frame(f'{key}/{kind}')
            view.update_frame(rgb, seq, self._stale(topic) and rgb is not None)

    def refresh_lidar(self):
        if self.lidar.scene is None:
            self.lidar.set_robot(self.ros.robot_description())
        self.lidar.update_robot(self.ros.all_joint_positions())
        xyz, seq = self.ros.latest_cloud()
        self.lidar.update_cloud(xyz, self.ros.cloud_frame(), seq, self._stale(self.cfg['lidar_topic']))

    def refresh_status(self):
        rows = [self.ros.monitor.status(t) for t, _, _ in self.cfg['status_topics']]
        self.status_table.setRowCount(len(rows))
        for r, st in enumerate(rows):
            items = [st.topic, st.state, f'{st.rate_hz:.1f} / {st.expected_hz:g}',
                     '-' if st.age_s is None else f'{st.age_s:.1f}']
            for c, text in enumerate(items):
                it = self.status_table.item(r, c)
                if it is None:
                    it = QtWidgets.QTableWidgetItem()
                    self.status_table.setItem(r, c, it)
                it.setText(text)
            self.status_table.item(r, 1).setForeground(QtGui.QBrush(QtGui.QColor(STATE_COLOURS.get(st.state, '#000'))))
        info = self.ros.monitor.status('/rtabmap/info')
        if info.state == NO_DATA:
            self.slam_label.setText('SLAM: not running (start bunker_aubo_slam slam.launch.py)')
        else:
            self.slam_label.setText(f'SLAM: {info.state} | loop closures detected: {self.ros.loop_closures()}')

    def closeEvent(self, event):
        self.stop_base()
        super().closeEvent(event)
