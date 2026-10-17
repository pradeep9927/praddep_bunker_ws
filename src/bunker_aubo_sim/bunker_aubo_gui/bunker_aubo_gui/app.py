import os
import signal
import sys

os.environ.setdefault('PYQTGRAPH_QT_LIB', 'PyQt5')

import rclpy  # noqa: E402
import yaml  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from PyQt5 import QtCore, QtWidgets  # noqa: E402


def load_config(path=None):
    if not path:
        path = os.path.join(get_package_share_directory('bunker_aubo_gui'), 'config', 'gui.yaml')
    with open(path) as f:
        return yaml.safe_load(f)


def main(args=None):
    rclpy.init(args=args)
    probe = rclpy.create_node('operator_gui_cfg')
    cfg_path = probe.declare_parameter('config', '').value
    probe.destroy_node()
    cfg = load_config(cfg_path)

    from .main_window import MainWindow   # after the Qt binding is chosen
    from .ros_interface import RosInterface

    app = QtWidgets.QApplication(sys.argv)
    ros = RosInterface(cfg)
    ros.start()
    win = MainWindow(ros, cfg)
    win.show()

    signal.signal(signal.SIGINT, lambda *_: app.quit())
    keepalive = QtCore.QTimer()
    keepalive.timeout.connect(lambda: None)
    keepalive.start(200)

    code = app.exec_()
    ros.shutdown()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
