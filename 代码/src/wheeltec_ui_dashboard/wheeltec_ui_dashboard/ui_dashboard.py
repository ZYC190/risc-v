import sys
import os
import json
import time
import math
import signal
import psutil
import numpy as np

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# When launched from SSH or a virtualenv, Qt can inherit X11 forwarding and open
# on the remote computer. Default to the robot's local touchscreen instead.
if os.environ.get("K1_DASHBOARD_REMOTE") != "1":
    uid = getattr(os, "getuid", lambda: 1000)()
    runtime_dir = os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{uid}")
    wayland_socket = os.path.join(runtime_dir, "wayland-0")
    if os.path.exists(wayland_socket):
        os.environ.pop("DISPLAY", None)
        os.environ.setdefault("XDG_RUNTIME_DIR", runtime_dir)
        os.environ.setdefault("WAYLAND_DISPLAY", "wayland-0")
        os.environ.setdefault("QT_QPA_PLATFORM", "wayland")
    else:
        os.environ["DISPLAY"] = os.environ.get("K1_DASHBOARD_DISPLAY", ":0")
        os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QStackedWidget,
    QGridLayout,
    QTextBrowser,
    QFrame,
    QProgressBar,
    QScrollArea,
    QScroller,
    QStyle,
)
from PyQt5.QtCore import QEvent, QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QImage, QPixmap, QPainter, QBrush, QColor, QPen


BG = "#08111f"
PANEL = "#0f172a"
PANEL_2 = "#152238"
TEXT = "#e8f2ff"
MUTED = "#91a4bd"
CYAN = "#24d8ff"
GREEN = "#39f2a6"
YELLOW = "#ffd166"
ORANGE = "#ff9f43"
RED = "#ff4d6d"
PURPLE = "#a78bfa"
BORDER = "#23344f"
SURFACE = "#111c30"
SURFACE_2 = "#17243a"


def asset_path(filename):
    try:
        package_share = get_package_share_directory("wheeltec_ui_dashboard")
        return os.path.join(package_share, "assets", filename)
    except PackageNotFoundError:
        package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(package_root, "assets", filename)


def card_style(border=CYAN, bg=PANEL):
    return f"""
        QFrame {{
            background-color: {bg};
            border: 1px solid {border};
            border-radius: 8px;
        }}
    """


def button_style(accent=CYAN, danger=False):
    hover = RED if danger else accent
    color = RED if danger else accent
    return f"""
        QPushButton {{
            background-color: {SURFACE};
            border: 1px solid {color};
            border-radius: 8px;
            color: {color};
            padding: 7px 10px;
            font-size: 14px;
            font-weight: 700;
        }}
        QPushButton:hover {{
            background-color: {hover};
            color: #06101d;
        }}
        QPushButton:pressed {{
            background-color: #ffffff;
            color: #06101d;
        }}
    """


def page_header(parent, title, subtitle, accent=CYAN):
    header = QFrame()
    header.setFixedHeight(66)
    header.setStyleSheet(card_style(BORDER, SURFACE))
    row = QHBoxLayout(header)
    row.setContentsMargins(12, 9, 12, 9)
    row.setSpacing(12)

    back = create_back_button(parent)
    back.setFixedSize(88, 42)
    row.addWidget(back)

    text_box = QVBoxLayout()
    text_box.setSpacing(1)
    title_text = QLabel(title)
    title_text.setStyleSheet(f"color:{TEXT}; font-size:21px; font-weight:900; border:none; background:transparent;")
    subtitle_text = QLabel(subtitle)
    subtitle_text.setStyleSheet(f"color:{MUTED}; font-size:12px; border:none; background:transparent;")
    text_box.addWidget(title_text)
    text_box.addWidget(subtitle_text)
    row.addLayout(text_box, stretch=1)

    badge = QLabel("ROS2")
    badge.setAlignment(Qt.AlignCenter)
    badge.setFixedSize(74, 34)
    badge.setStyleSheet(f"""
        QLabel {{
            color:{accent};
            background:#0b1628;
            border:1px solid {accent};
            border-radius:8px;
            font-size:13px;
            font-weight:900;
        }}
    """)
    row.addWidget(badge)
    return header


def title_label(text, size=24, color=TEXT):
    label = QLabel(text)
    label.setStyleSheet(f"color:{color}; font-size:{size}px; font-weight:800;")
    return label


def small_label(text, color=MUTED):
    label = QLabel(text)
    label.setStyleSheet(f"color:{color}; font-size:12px;")
    return label


class TouchScrollArea(QScrollArea):
    """Scrolls with both native touch gestures and touch-synthesized mouse drags."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._drag_origin_y = None
        self._drag_origin_value = 0
        self._dragging = False
        app = QApplication.instance()
        if app is not None:
            app.installEventFilter(self)

    def eventFilter(self, watched, event):
        if not self._belongs_to_scroll_area(watched) or self._belongs_to_button(watched):
            return super().eventFilter(watched, event)

        event_type = event.type()
        if event_type == QEvent.MouseButtonPress and event.button() == Qt.LeftButton:
            self._start_drag(event.globalPos().y())
        elif event_type == QEvent.MouseMove and self._drag_origin_y is not None:
            if event.buttons() & Qt.LeftButton and self._move_drag(event.globalPos().y()):
                event.accept()
                return True
        elif event_type == QEvent.MouseButtonRelease and event.button() == Qt.LeftButton:
            was_dragging = self._finish_drag()
            if was_dragging:
                event.accept()
                return True
        elif event_type == QEvent.TouchBegin:
            point = self._touch_y(event)
            if point is not None:
                self._start_drag(point)
        elif event_type == QEvent.TouchUpdate and self._drag_origin_y is not None:
            point = self._touch_y(event)
            if point is not None and self._move_drag(point):
                event.accept()
                return True
        elif event_type in (QEvent.TouchEnd, QEvent.TouchCancel):
            was_dragging = self._finish_drag()
            if was_dragging:
                event.accept()
                return True

        return super().eventFilter(watched, event)

    def _start_drag(self, y):
        self._drag_origin_y = float(y)
        self._drag_origin_value = self.verticalScrollBar().value()
        self._dragging = False

    def _move_drag(self, y):
        distance = float(y) - self._drag_origin_y
        if abs(distance) >= 8:
            self._dragging = True
        if self._dragging:
            self.verticalScrollBar().setValue(
                self._drag_origin_value - int(distance)
            )
        return self._dragging

    def _finish_drag(self):
        was_dragging = self._dragging
        self._drag_origin_y = None
        self._dragging = False
        return was_dragging

    @staticmethod
    def _touch_y(event):
        points = event.touchPoints()
        return points[0].screenPos().y() if points else None

    def _belongs_to_scroll_area(self, watched):
        widget = watched if isinstance(watched, QWidget) else None
        while widget is not None:
            if widget in (self, self.viewport(), self.widget()):
                return True
            widget = widget.parentWidget()
        return False

    def _belongs_to_button(self, watched):
        widget = watched if isinstance(watched, QWidget) else None
        while widget is not None and widget is not self:
            if isinstance(widget, QPushButton):
                return True
            widget = widget.parentWidget()
        return False


class CoverImageLabel(QLabel):
    def __init__(self, image_name):
        super().__init__()
        self.source_pixmap = QPixmap(asset_path(image_name))
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumHeight(190)
        self.setStyleSheet("background:transparent; border:none;")

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#07111e"))
        if self.source_pixmap.isNull() or self.width() <= 0 or self.height() <= 0:
            return
        scaled = self.source_pixmap.scaled(
            self.size(),
            Qt.KeepAspectRatioByExpanding,
            Qt.SmoothTransformation,
        )
        x = (self.width() - scaled.width()) // 2
        y = (self.height() - scaled.height()) // 2
        painter.drawPixmap(x, y, scaled)
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(QColor(BORDER), 1))
        painter.drawRoundedRect(self.rect().adjusted(0, 0, -1, -1), 7, 7)


class Ros2EngineThread(QThread):
    map_signal = pyqtSignal(object, float, float, float)
    pose_signal = pyqtSignal(float, float)
    air_data_signal = pyqtSignal(dict)
    voice_log_signal = pyqtSignal(str)
    status_signal = pyqtSignal(str, str)
    arm_log_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        if not rclpy.ok():
            rclpy.init()
        self.node = Node("k1_ui_dashboard")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.node.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)
        self.node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10)
        self.node.create_subscription(String, "/air_sensor_data", self.air_callback, 10)
        self.node.create_subscription(String, "/voice_log", self.voice_callback, 10)
        self.node.create_subscription(String, "/feedback_words", self.feedback_callback, 10)
        self.node.create_subscription(String, "/actionstatus", self.action_status_callback, 10)
        self.node.create_subscription(String, "/arm_status", self.arm_status_callback, 10)
        self.node.create_subscription(String, "/esp32_status", self.esp32_status_callback, 10)

        self.goal_pub = self.node.create_publisher(PoseStamped, "/goal_pose", 10)
        self.esp32_pub = self.node.create_publisher(String, "/esp32_cmd", 10)
        self.voice_cmd_pub = self.node.create_publisher(String, "/voice_trigger", 10)
        self.arm_cmd_pub = self.node.create_publisher(String, "/arm_cmd", 10)
        self.cmd_vel_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.security_alert_pub = self.node.create_publisher(
            String, "/home/security/alert", 10
        )
        self.air_refresh_pub = self.node.create_publisher(
            String, "/air_sensor_refresh", 10
        )

    def map_callback(self, msg):
        width, height = msg.info.width, msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        img_data = np.full((height, width, 3), 14, dtype=np.uint8)
        img_data[data == -1] = [12, 20, 34]
        img_data[data == 0] = [225, 235, 244]
        img_data[data >= 50] = [22, 35, 55]
        img_data = np.flipud(img_data)
        self.map_signal.emit(img_data, ox, oy, res)

    def pose_callback(self, msg):
        self.pose_signal.emit(msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.status_signal.emit("pose", f"X {msg.pose.pose.position.x:.2f} / Y {msg.pose.pose.position.y:.2f}")

    def air_callback(self, msg):
        try:
            data_dict = json.loads(msg.data)
            self.air_data_signal.emit(data_dict)
            self.status_signal.emit("air", "环境数据在线")
        except Exception:
            self.status_signal.emit("air", "环境数据格式异常")

    def voice_callback(self, msg):
        self.voice_log_signal.emit(msg.data)
        self.status_signal.emit("voice", "语音在线")

    def feedback_callback(self, msg):
        self.voice_log_signal.emit(msg.data)

    def action_status_callback(self, msg):
        self.status_signal.emit("task", msg.data)

    def arm_status_callback(self, msg):
        self.arm_log_signal.emit(msg.data)
        self.status_signal.emit("arm", msg.data[:24])

    def esp32_status_callback(self, msg):
        self.status_signal.emit("esp32", msg.data[:24])

    def send_goal(self, target_x, target_y, oz=0.0, ow=1.0):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = float(target_x)
        goal_msg.pose.position.y = float(target_y)
        goal_msg.pose.orientation.z = float(oz)
        goal_msg.pose.orientation.w = float(ow)
        self.goal_pub.publish(goal_msg)
        self.status_signal.emit("task", f"导航目标已下发 ({target_x:.2f},{target_y:.2f})")

    def publish_string(self, pub, payload):
        msg = String()
        msg.data = payload
        pub.publish(msg)

    def send_esp32_cmd(self, cmd):
        self.publish_string(self.esp32_pub, cmd)
        self.status_signal.emit("esp32", f"指令: {cmd}")

    def request_air_refresh(self):
        self.publish_string(self.air_refresh_pub, "REFRESH")
        self.status_signal.emit("air", "正在请求最新传感器数据")

    def send_arm_cmd(self, cmd):
        self.publish_string(self.arm_cmd_pub, cmd)
        self.arm_log_signal.emit(f"已下发机械臂指令: {cmd}")
        self.status_signal.emit("arm", f"指令: {cmd}")

    def send_voice_trigger(self):
        self.publish_string(self.voice_cmd_pub, "TOGGLE_LISTENING")

    def set_voice_listening(self, enabled):
        command = "START_LISTENING" if enabled else "STOP_LISTENING"
        self.publish_string(self.voice_cmd_pub, command)
        state = "正在聆听现场讲话" if enabled else "现场讲话已暂停"
        self.status_signal.emit("voice", state)

    def send_emergency_sos(self):
        payload = {
            "type": "emergency_sos",
            "level": "emergency",
            "title": "机器人紧急求救",
            "message": "现场人员通过机器人触摸屏发起紧急求救",
            "location": "机器人当前位置",
            "abnormal": "触摸屏紧急求救按钮已触发",
            "suggestion": "请立即联系现场人员并通过家庭地图查看机器人位置",
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        }
        self.publish_string(
            self.security_alert_pub,
            json.dumps(payload, ensure_ascii=False),
        )
        self.status_signal.emit("task", "紧急求救已发送给家长")

    def emergency_stop(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.send_arm_cmd("STOP")
        self.send_esp32_cmd("ALL_OFF")
        self.status_signal.emit("task", "急停：底盘零速度 / 机械臂停止 / 物联关闭")

    def run(self):
        try:
            self.executor.spin()
        except ExternalShutdownException:
            pass
        finally:
            self.executor.remove_node(self.node)
            self.node.destroy_node()

    def stop(self):
        self.executor.shutdown(timeout_sec=2.0)
        if rclpy.ok():
            rclpy.shutdown()
        self.requestInterruption()


def create_back_button(parent):
    btn = QPushButton("返回")
    btn.setFixedHeight(38)
    btn.setStyleSheet(button_style(MUTED))
    btn.clicked.connect(parent.go_back)
    return btn


class StatCard(QFrame):
    def __init__(self, title, value="--", unit="", accent=CYAN):
        super().__init__()
        self.accent = accent
        self.setMinimumHeight(88)
        self.setStyleSheet(card_style(accent))
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 12, 16, 12)
        layout.setSpacing(4)
        self.title = QLabel(title)
        self.title.setStyleSheet(f"color:{MUTED}; font-size:12px; border:none; background:transparent;")
        self.value = QLabel(value)
        self.value.setStyleSheet(f"color:{accent}; font-size:24px; font-weight:900; border:none; background:transparent;")
        self.unit = QLabel(unit)
        self.unit.setStyleSheet(f"color:{MUTED}; font-size:10px; border:none; background:transparent;")
        layout.addWidget(self.title)
        layout.addWidget(self.value)
        layout.addWidget(self.unit)

    def set_value(self, value, accent=None):
        self.value.setText(str(value))
        if accent:
            self.value.setStyleSheet(f"color:{accent}; font-size:24px; font-weight:900; border:none; background:transparent;")


class DashboardStatusCard(QFrame):
    def __init__(self, title, value="待命", accent=CYAN):
        super().__init__()
        self.accent = accent
        self.setMinimumHeight(72)
        self.setStyleSheet(card_style(BORDER, SURFACE))
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 10, 14, 10)
        layout.setSpacing(4)

        self.title = QLabel(title)
        self.title.setStyleSheet(f"color:{MUTED}; font-size:12px; border:none; background:transparent;")
        self.value = QLabel(value)
        self.value.setWordWrap(True)
        self.value.setStyleSheet(f"color:{TEXT}; font-size:16px; font-weight:800; border:none; background:transparent;")
        self.line = QFrame()
        self.line.setFixedHeight(3)
        self.line.setStyleSheet(f"background:{accent}; border:none; border-radius:1px;")

        layout.addWidget(self.title)
        layout.addWidget(self.value, stretch=1)
        layout.addWidget(self.line)

    def set_value(self, value, accent=None):
        self.value.setText(str(value))
        if accent:
            self.line.setStyleSheet(f"background:{accent}; border:none; border-radius:1px;")


class ServiceImageTile(QPushButton):
    def __init__(self, title, description, image_name, accent=CYAN, danger=False):
        super().__init__()
        border_color = RED if danger else accent
        self.setText("")
        self.setMinimumHeight(84)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet(f"""
            QPushButton {{
                background-color:{SURFACE};
                border:1px solid {BORDER};
                border-left:5px solid {border_color};
                border-radius:8px;
                padding:0px;
            }}
            QPushButton:hover {{
                background-color:{SURFACE_2};
                border:1px solid {border_color};
                border-left:5px solid {border_color};
            }}
            QPushButton:pressed {{
                background-color:{border_color};
            }}
        """)

        row = QHBoxLayout(self)
        row.setContentsMargins(7, 6, 13, 6)
        row.setSpacing(14)

        image_label = QLabel()
        image_label.setFixedSize(108, 108)
        image_label.setAlignment(Qt.AlignCenter)
        image_label.setStyleSheet(
            f"background:#07111e; border:1px solid {BORDER}; border-radius:7px;"
        )
        pixmap = QPixmap(asset_path(image_name))
        if not pixmap.isNull():
            image_label.setPixmap(
                pixmap.scaled(106, 106, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )

        text_column = QVBoxLayout()
        text_column.setSpacing(4)
        title_label_widget = QLabel(title)
        title_label_widget.setWordWrap(True)
        title_label_widget.setStyleSheet(
            f"color:{TEXT}; font-size:21px; font-weight:900; border:none; background:transparent;"
        )
        description_label = QLabel(description)
        description_label.setWordWrap(True)
        description_label.setStyleSheet(
            f"color:{accent if not danger else RED}; font-size:15px; font-weight:800; "
            "border:none; background:transparent;"
        )
        enter_label = QLabel("进入  →")
        enter_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        enter_label.setStyleSheet(
            f"color:{accent if not danger else RED}; font-size:13px; font-weight:900; "
            "border:none; background:transparent;"
        )
        text_column.addWidget(title_label_widget)
        text_column.addWidget(description_label)
        text_column.addStretch()
        text_column.addWidget(enter_label)

        for label in (image_label, title_label_widget, description_label, enter_label):
            label.setAttribute(Qt.WA_TransparentForMouseEvents, True)

        row.addWidget(image_label)
        row.addLayout(text_column, stretch=1)


class ServiceAppTile(QPushButton):
    def __init__(self, title, description, image_name, accent):
        super().__init__()
        self.setText("")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet(f"""
            QPushButton {{
                background:{SURFACE};
                border:1px solid {BORDER};
                border-radius:8px;
                padding:0px;
            }}
            QPushButton:hover {{ border:2px solid {accent}; background:{SURFACE_2}; }}
            QPushButton:pressed {{ background:#1c2d46; }}
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 8)
        layout.setSpacing(5)

        image = CoverImageLabel(image_name)
        image.setMinimumHeight(82)
        title_row = QHBoxLayout()
        title_row.setContentsMargins(11, 0, 10, 0)
        title_label_widget = QLabel(title)
        title_label_widget.setStyleSheet(
            f"color:{TEXT}; font-size:19px; font-weight:900; border:none; background:transparent;"
        )
        arrow = QLabel("→")
        arrow.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        arrow.setStyleSheet(
            f"color:{accent}; font-size:19px; font-weight:900; border:none; background:transparent;"
        )
        title_row.addWidget(title_label_widget, stretch=1)
        title_row.addWidget(arrow)
        description_label = QLabel(description)
        description_label.setContentsMargins(11, 0, 10, 0)
        description_label.setStyleSheet(
            f"color:{accent}; font-size:12px; font-weight:800; border:none; background:transparent;"
        )
        for child in (image, title_label_widget, arrow, description_label):
            child.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        layout.addWidget(image, stretch=1)
        layout.addLayout(title_row)
        layout.addWidget(description_label)


class HubModuleButton(QPushButton):
    def __init__(self, title, subtitle, detail, accent, image_name):
        super().__init__()
        self.setText("")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumHeight(250)
        self.setStyleSheet(f"""
            QPushButton {{
                background-color:{SURFACE};
                border:1px solid {BORDER};
                border-top:5px solid {accent};
                border-radius:8px;
                padding:0px;
            }}
            QPushButton:hover {{
                background-color:{SURFACE_2};
                border:1px solid {accent};
                border-top:5px solid {accent};
            }}
            QPushButton:pressed {{
                background-color:#1c2d46;
            }}
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(18, 13, 18, 15)
        layout.setSpacing(6)

        image_label = QLabel()
        image_label.setFixedSize(116, 116)
        image_label.setAlignment(Qt.AlignCenter)
        image_label.setStyleSheet(
            f"background:#07111e; border:1px solid {BORDER}; border-radius:8px;"
        )
        pixmap = QPixmap(asset_path(image_name))
        if not pixmap.isNull():
            image_label.setPixmap(
                pixmap.scaled(114, 114, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )

        title_label_widget = QLabel(title)
        title_label_widget.setStyleSheet(
            f"color:{TEXT}; font-size:25px; font-weight:900; border:none; background:transparent;"
        )
        subtitle_label = QLabel(subtitle)
        subtitle_label.setWordWrap(True)
        subtitle_label.setStyleSheet(
            f"color:{accent}; font-size:16px; font-weight:800; border:none; background:transparent;"
        )
        detail_label = QLabel(detail)
        detail_label.setWordWrap(True)
        detail_label.setStyleSheet(
            f"color:{MUTED}; font-size:13px; border:none; background:transparent;"
        )
        enter_label = QLabel("进入  →")
        enter_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        enter_label.setStyleSheet(
            f"color:{accent}; font-size:15px; font-weight:900; border:none; background:transparent;"
        )

        for label in (image_label, title_label_widget, subtitle_label, detail_label, enter_label):
            label.setAttribute(Qt.WA_TransparentForMouseEvents, True)

        layout.addWidget(image_label, alignment=Qt.AlignHCenter)
        layout.addWidget(title_label_widget)
        layout.addWidget(subtitle_label)
        layout.addWidget(detail_label)
        layout.addStretch()
        layout.addWidget(enter_label)


class AnimatedEyesWidget(QWidget):
    """Full-screen animated eyes for the robot's standby state."""

    activated = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.t = 0.0
        self.blink = 1.0
        self.setMinimumSize(1, 1)
        self.setAttribute(Qt.WA_AcceptTouchEvents, True)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(33)

    def tick(self):
        self.t += 0.055
        phase = self.t % 6.4
        if 5.72 <= phase <= 6.08:
            self.blink = max(0.05, min(1.0, abs(phase - 5.90) / 0.18))
        else:
            self.blink = 1.0
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.activated.emit()
        super().mousePressEvent(event)

    def event(self, event):
        if event.type() == QEvent.TouchBegin:
            self.activated.emit()
            return True
        return super().event(event)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w * 0.50, h * 0.48
        painter.fillRect(self.rect(), QColor(4, 10, 18))

        eye_w = min(int(w * 0.34), 310)
        eye_h_base = min(int(h * 0.48), 225)
        eye_h = max(12, int(eye_h_base * self.blink))
        eye_centers = (w * 0.27, w * 0.73)
        look_x = math.sin(self.t * 0.72) * min(22, eye_w * 0.08)
        look_y = math.sin(self.t * 0.43) * min(13, eye_h_base * 0.07)

        for ex in eye_centers:
            glow_w = eye_w + 18
            glow_h = eye_h + 18
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(QColor(36, 216, 255, 34)))
            painter.drawEllipse(
                int(ex - glow_w / 2),
                int(cy - glow_h / 2),
                glow_w,
                glow_h,
            )
            painter.setBrush(QBrush(QColor(238, 250, 255)))
            painter.drawEllipse(
                int(ex - eye_w / 2),
                int(cy - eye_h / 2),
                eye_w,
                eye_h,
            )

            if self.blink > 0.24:
                iris = min(int(eye_w * 0.42), int(eye_h_base * 0.68))
                pupil = max(24, int(iris * 0.48))
                painter.setBrush(QBrush(QColor(52, 202, 238)))
                painter.drawEllipse(
                    int(ex - iris / 2 + look_x),
                    int(cy - iris / 2 + look_y),
                    iris,
                    iris,
                )
                painter.setBrush(QBrush(QColor(4, 14, 24)))
                painter.drawEllipse(
                    int(ex - pupil / 2 + look_x),
                    int(cy - pupil / 2 + look_y),
                    pupil,
                    pupil,
                )
                shine = max(12, int(iris * 0.20))
                painter.setBrush(QBrush(QColor(255, 255, 255)))
                painter.drawEllipse(
                    int(ex + iris * 0.08 + look_x),
                    int(cy - iris * 0.34 + look_y),
                    shine,
                    shine,
                )
                painter.setBrush(QBrush(QColor(255, 255, 255, 180)))
                painter.drawEllipse(
                    int(ex - iris * 0.27 + look_x),
                    int(cy + iris * 0.15 + look_y),
                    max(6, shine // 2),
                    max(6, shine // 2),
                )

            painter.setPen(QPen(QColor(96, 224, 255, 170), 4, Qt.SolidLine, Qt.RoundCap))
            painter.setBrush(Qt.NoBrush)
            brow_y = cy - eye_h_base * 0.70
            painter.drawArc(
                int(ex - eye_w * 0.28),
                int(brow_y),
                int(eye_w * 0.56),
                int(eye_h_base * 0.34),
                25 * 16,
                130 * 16,
            )

        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(255, 105, 145, 115)))
        cheek_w = max(44, int(w * 0.075))
        cheek_h = max(20, int(h * 0.055))
        painter.drawEllipse(int(w * 0.08), int(h * 0.72), cheek_w, cheek_h)
        painter.drawEllipse(int(w * 0.92 - cheek_w), int(h * 0.72), cheek_w, cheek_h)

        painter.setPen(QPen(QColor(57, 242, 166), 6, Qt.SolidLine, Qt.RoundCap))
        mouth_w = min(120, int(w * 0.15))
        painter.drawArc(
            int(cx - mouth_w / 2),
            int(h * 0.70),
            mouth_w,
            max(42, int(h * 0.12)),
            200 * 16,
            140 * 16,
        )


class EyeHomePage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        self.eyes = AnimatedEyesWidget()
        self.eyes.activated.connect(self.enter_dashboard)
        root.addWidget(self.eyes)

        self.menu_button = self.create_menu_button()

    def create_menu_button(self):
        btn = QPushButton("≡", self)
        btn.setToolTip("进入控制界面")
        btn.setFixedSize(52, 52)
        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: rgba(10,24,40,0.88);
                border: 1px solid {CYAN};
                border-radius: 25px;
                color: {TEXT};
                font-size: 30px;
                font-weight: 800;
                padding: 0px;
            }}
            QPushButton:hover {{
                background-color: {CYAN};
                color: #06101d;
            }}
        """)
        btn.clicked.connect(self.enter_dashboard)
        btn.raise_()
        return btn

    def enter_dashboard(self):
        self.main_window.switch_to_page(1)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.place_menu_button()

    def showEvent(self, event):
        super().showEvent(event)
        self.place_menu_button()

    def place_menu_button(self):
        screen_w = self.width()
        screen = self.window().screen()
        if screen:
            screen_w = min(screen_w, screen.geometry().width())
        margin = 16
        self.menu_button.move(screen_w - self.menu_button.width() - margin, margin)
        self.menu_button.raise_()


class MainMenuPage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        root = QHBoxLayout(self)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(10)

        left = QVBoxLayout()
        left.setSpacing(10)

        identity = QFrame()
        identity.setStyleSheet(card_style(CYAN, SURFACE))
        identity_layout = QVBoxLayout(identity)
        identity_layout.setContentsMargins(16, 12, 16, 12)
        identity_layout.setSpacing(6)
        robot_image = CoverImageLabel("indoor_patrol.png")
        robot_image.setMinimumHeight(108)
        clock_row = QHBoxLayout(robot_image)
        clock_row.setContentsMargins(8, 8, 8, 8)
        clock_row.addStretch()
        self.clock = QLabel()
        self.clock.setAlignment(Qt.AlignCenter)
        self.clock.setFixedSize(92, 30)
        self.clock.setStyleSheet(f"""
            color:{CYAN}; font-size:16px; font-weight:900;
            background:rgba(4,10,18,0.86); border:1px solid {CYAN}; border-radius:7px;
        """)
        clock_row.addWidget(self.clock, alignment=Qt.AlignTop)
        identity_layout.addWidget(robot_image, stretch=1)
        title = QLabel("家庭守护机器人")
        title.setWordWrap(True)
        title.setStyleSheet(
            f"color:{TEXT}; font-size:26px; font-weight:900; border:none; background:transparent;"
        )
        subtitle = QLabel("室内安全巡查 · 家庭状态守护 · 家人陪伴")
        subtitle.setWordWrap(True)
        subtitle.setStyleSheet(
            f"color:{MUTED}; font-size:12px; border:none; background:transparent;"
        )
        identity_layout.addWidget(title)
        identity_layout.addWidget(subtitle)
        identity_layout.addStretch()
        left.addWidget(identity, stretch=5)

        status_button = self._create_status_button()
        status_button.clicked.connect(
            lambda: self.main_window.open_module("status")
        )
        self.status_temperature_label = status_button.temperature_label
        self.status_humidity_label = status_button.humidity_label
        left.addWidget(status_button, stretch=4)

        services_button = self._create_module_button(
            "家庭服务",
            "地图 · 巡查 · 预警 · 家居 · 记录 · 聊天",
            "进入家庭守护服务中心",
            GREEN,
            "family_services.png",
        )
        services_button.clicked.connect(
            lambda: self.main_window.open_module("services")
        )

        root.addLayout(left, stretch=4)
        root.addWidget(services_button, stretch=7)

        self.health_badge = QLabel("守护待命", identity)
        self.health_badge.setAlignment(Qt.AlignCenter)
        self.health_badge.setFixedSize(94, 30)
        self.health_badge.setStyleSheet(self._health_style(GREEN, False))
        identity_layout.addWidget(self.health_badge, alignment=Qt.AlignLeft)
        self.activity_line = QLabel("家庭守护系统已就绪")

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_clock)
        self.timer.start(1000)
        self.update_clock()

    def _create_status_button(self):
        button = QPushButton()
        button.setText("")
        button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        button.setStyleSheet(f"""
            QPushButton {{
                background:{SURFACE}; border:1px solid {BORDER};
                border-left:5px solid {CYAN}; border-radius:8px; padding:0px;
            }}
            QPushButton:hover {{ background:{SURFACE_2}; border-color:{CYAN}; }}
            QPushButton:pressed {{ background:#1c2d46; }}
        """)
        layout = QVBoxLayout(button)
        layout.setContentsMargins(12, 10, 12, 9)
        layout.setSpacing(6)
        title_row = QHBoxLayout()
        title = QLabel("家庭状态")
        title.setStyleSheet(
            f"color:{TEXT}; font-size:23px; font-weight:900; border:none; background:transparent;"
        )
        live = QLabel("实时")
        live.setAlignment(Qt.AlignCenter)
        live.setFixedSize(48, 26)
        live.setStyleSheet(
            f"color:{GREEN}; background:#10251f; border:1px solid {GREEN}; border-radius:7px; font-weight:900;"
        )
        title_row.addWidget(title)
        title_row.addStretch()
        title_row.addWidget(live)

        status_body = QHBoxLayout()
        status_body.setSpacing(10)
        status_image = CoverImageLabel("system_status.png")
        status_image.setMinimumSize(108, 1)
        status_image.setMaximumWidth(145)
        status_image.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        status_body.addWidget(status_image, stretch=4)

        status_text = QVBoxLayout()
        status_text.setSpacing(4)
        monitor = QLabel("环境实时监测")
        monitor.setStyleSheet(
            f"color:{MUTED}; font-size:13px; font-weight:700; border:none; background:transparent;"
        )
        button.temperature_label = QLabel("温度 -- ℃")
        button.humidity_label = QLabel("湿度 -- %")
        button.temperature_label.setStyleSheet(
            f"color:{CYAN}; font-size:18px; font-weight:900; border:none; background:transparent;"
        )
        button.humidity_label.setStyleSheet(
            f"color:{GREEN}; font-size:18px; font-weight:900; border:none; background:transparent;"
        )
        status_text.addWidget(monitor)
        status_text.addWidget(button.temperature_label)
        status_text.addWidget(button.humidity_label)
        status_text.addStretch()
        status_body.addLayout(status_text, stretch=5)

        enter = QLabel("查看六项数据与历史趋势  →")
        enter.setAlignment(Qt.AlignRight)
        enter.setStyleSheet(
            f"color:{CYAN}; font-size:12px; font-weight:800; border:none; background:transparent;"
        )
        for child in (
            title,
            live,
            monitor,
            button.temperature_label,
            button.humidity_label,
            enter,
        ):
            child.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        layout.addLayout(title_row)
        layout.addLayout(status_body, stretch=1)
        layout.addWidget(enter)
        return button

    def _create_module_button(
        self, title, subtitle, detail, accent, image_name, compact=False
    ):
        button = QPushButton()
        button.setText("")
        button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        button.setStyleSheet(f"""
            QPushButton {{
                background:{SURFACE};
                border:1px solid {BORDER};
                border-left:5px solid {accent};
                border-radius:8px;
                padding:0px;
            }}
            QPushButton:hover {{ background:{SURFACE_2}; border-color:{accent}; }}
            QPushButton:pressed {{ background:#1c2d46; }}
        """)
        layout = QVBoxLayout(button)
        layout.setContentsMargins(16, 13, 16, 13)
        layout.setSpacing(6)
        if compact:
            image = QLabel()
            image_size = 106
            image.setFixedSize(image_size, image_size)
            image.setAlignment(Qt.AlignCenter)
            image.setStyleSheet(
                f"background:#07111e; border:1px solid {BORDER}; border-radius:8px;"
            )
            pixmap = QPixmap(asset_path(image_name))
            if not pixmap.isNull():
                image.setPixmap(
                    pixmap.scaled(
                        image_size - 2,
                        image_size - 2,
                        Qt.KeepAspectRatio,
                        Qt.SmoothTransformation,
                    )
                )
        else:
            image = CoverImageLabel(image_name)
        title_label_widget = QLabel(title)
        title_label_widget.setWordWrap(True)
        title_label_widget.setStyleSheet(
            f"color:{TEXT}; font-size:{22 if compact else 36}px; font-weight:900; border:none; background:transparent;"
        )
        subtitle_label = QLabel(subtitle)
        subtitle_label.setWordWrap(True)
        subtitle_label.setStyleSheet(
            f"color:{accent}; font-size:{14 if compact else 19}px; font-weight:800; border:none; background:transparent;"
        )
        detail_label = QLabel(detail)
        detail_label.setWordWrap(True)
        detail_label.setStyleSheet(
            f"color:{MUTED}; font-size:12px; border:none; background:transparent;"
        )
        enter_label = QLabel("进入  →")
        enter_label.setAlignment(Qt.AlignRight)
        enter_label.setStyleSheet(
            f"color:{accent}; font-size:14px; font-weight:900; border:none; background:transparent;"
        )
        for child in (image, title_label_widget, subtitle_label, detail_label, enter_label):
            child.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        if compact:
            layout.addWidget(image, alignment=Qt.AlignHCenter)
        else:
            layout.addWidget(image, stretch=1)
        layout.addWidget(title_label_widget)
        layout.addWidget(subtitle_label)
        if not compact:
            layout.addWidget(detail_label)
        layout.addStretch()
        layout.addWidget(enter_label)
        button.subtitle_label = subtitle_label
        button.detail_label = detail_label
        return button

    def _health_style(self, accent, alert):
        return f"""
            QLabel {{
                color:{accent};
                background:{'#2a1119' if alert else '#10251f'};
                border:1px solid {accent};
                border-radius:8px;
                font-size:12px;
                font-weight:800;
            }}
        """

    def update_clock(self):
        self.clock.setText(time.strftime("%H:%M:%S"))

    def update_environment(self, data):
        temperature = data.get("温度", data.get("temperature", data.get("temp")))
        humidity = data.get("湿度", data.get("humidity", data.get("humi")))
        if temperature is None and humidity is None:
            return
        temperature_text = "--" if temperature is None else str(temperature)
        humidity_text = "--" if humidity is None else str(humidity)
        self.status_temperature_label.setText(f"温度 {temperature_text} ℃")
        self.status_humidity_label.setText(f"湿度 {humidity_text} %")

    def update_status(self, key, value):
        labels = {
            "task": "巡查任务",
            "pose": "当前位置",
            "air": "环境安全",
            "esp32": "家居设备",
            "arm": "生活协助",
            "voice": "看护互动",
        }
        if key not in labels:
            return
        alert = any(flag in value for flag in ("异常", "急停", "STOP"))
        accent = RED if alert else GREEN
        self.health_badge.setText("需要关注" if alert else "守护在线")
        self.health_badge.setStyleSheet(self._health_style(accent, alert))
        self.activity_line.setText(f"{labels[key]}  ·  {value}")
        self.health_badge.setToolTip(self.activity_line.text())


class GuardianServicesPage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(9)
        layout.addWidget(page_header(main_window, "家庭服务", "家庭守护机器人服务中心", GREEN))

        grid = QGridLayout()
        grid.setSpacing(9)
        services = [
            ("家庭地图", "选择客厅、厨房或卧室", "indoor_map.png", "map", CYAN),
            ("家长聊天", "现场说话与家长消息", "child_care.png", "chat", GREEN),
            ("室内巡查", "按房间执行家庭巡查", "indoor_patrol.png", "patrol", GREEN),
            ("安全预警", "查看家庭异常与状态", "safety_alert.png", "safety", RED),
            ("家居联动", "控制灯光、通风和报警", "smart_home.png", "smart_home", ORANGE),
            ("看护记录", "回看机器人与家庭对话", "system_status.png", "records", PURPLE),
        ]
        for i, (title_text, desc, image_name, service_name, accent) in enumerate(services):
            button = ServiceAppTile(
                title_text,
                desc,
                image_name,
                accent,
            )
            button.clicked.connect(
                lambda checked=False, name=service_name: main_window.open_service(name)
            )
            grid.addWidget(button, i // 3, i % 3)
        for row in range(2):
            grid.setRowStretch(row, 1)
        layout.addLayout(grid, stretch=1)


class EnvironmentMetricCard(QFrame):
    def __init__(self, title, unit, accent):
        super().__init__()
        self.unit = unit
        self.accent = accent
        self.setStyleSheet(card_style(BORDER, SURFACE))
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(2)
        self.title_label = QLabel(title)
        self.title_label.setStyleSheet(
            f"color:{TEXT}; font-size:16px; font-weight:900; border:none; background:transparent;"
        )
        self.value_label = QLabel(f"-- {unit}".strip())
        self.value_label.setStyleSheet(
            f"color:{accent}; font-size:22px; font-weight:900; border:none; background:transparent;"
        )
        layout.addWidget(self.title_label)
        layout.addWidget(self.value_label, stretch=1)

    def set_value(self, value):
        text = "--" if value is None else str(value)
        self.value_label.setText(f"{text} {self.unit}".strip())


class EnvironmentTrendWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.history = []
        self.metric_key = "temperature"
        self.metric_label = "温度"
        self.metric_color = QColor(CYAN)
        self.setMinimumSize(250, 150)

    def set_history(self, history):
        self.history = list(history)[-43200:]
        self.update()

    def set_metric(self, key, label, color):
        self.metric_key = key
        self.metric_label = label
        self.metric_color = QColor(color)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(SURFACE))
        width, height = self.width(), self.height()
        left, top, right, bottom = 14, 34, width - 14, height - 22

        painter.setPen(QPen(QColor(BORDER), 1))
        for row in range(4):
            y = top + (bottom - top) * row / 3
            painter.drawLine(left, int(y), right, int(y))

        samples = self.history
        if len(samples) > 240:
            step = max(1, len(samples) // 240)
            samples = samples[::step]
        if len(samples) < 2:
            painter.setPen(QColor(MUTED))
            painter.drawText(self.rect(), Qt.AlignCenter, "等待环境历史数据...")
            return

        self._draw_series(
            painter,
            samples,
            self.metric_key,
            self.metric_color,
            left,
            top,
            right,
            bottom,
        )
        painter.setPen(self.metric_color)
        painter.drawText(left, 20, self.metric_label)
        painter.setPen(QColor(MUTED))
        painter.drawText(right - 90, 20, f"{len(samples)} 条记录")

    def _draw_series(self, painter, samples, key, color, left, top, right, bottom):
        values = []
        for sample in samples:
            try:
                values.append(float(sample[key]))
            except (KeyError, TypeError, ValueError):
                values.append(None)
        valid = [value for value in values if value is not None]
        if len(valid) < 2:
            return
        low, high = min(valid), max(valid)
        padding = max((high - low) * 0.15, 0.8)
        low -= padding
        high += padding
        painter.setPen(QPen(color, 3, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        previous = None
        count = max(1, len(values) - 1)
        for index, value in enumerate(values):
            if value is None:
                previous = None
                continue
            x = left + (right - left) * index / count
            ratio = (value - low) / max(high - low, 0.001)
            y = bottom - (bottom - top) * ratio
            point = (int(x), int(y))
            if previous is not None:
                painter.drawLine(previous[0], previous[1], point[0], point[1])
            previous = point


class _LegacyFamilyStatusPage(QWidget):
    HISTORY_FILE = os.path.expanduser(
        "~/.config/wheeltec_ui_dashboard/environment_history.json"
    )

    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        self.history = self._load_history()
        self.last_history_time = (
            float(self.history[-1].get("timestamp", 0)) if self.history else 0.0
        )

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(9)
        layout.addWidget(
            page_header(main_window, "家庭状态", "实时环境监测与历史记录", CYAN)
        )

        summary = QFrame()
        summary.setFixedHeight(104)
        summary.setStyleSheet(card_style(CYAN, SURFACE))
        summary_layout = QHBoxLayout(summary)
        summary_layout.setContentsMargins(8, 7, 16, 7)
        summary_layout.setSpacing(14)

        summary_image = QLabel()
        summary_image.setFixedSize(90, 90)
        summary_image.setAlignment(Qt.AlignCenter)
        summary_image.setStyleSheet(
            f"background:#07111e; border:1px solid {BORDER}; border-radius:8px;"
        )
        summary_pixmap = QPixmap(asset_path("system_status.png"))
        if not summary_pixmap.isNull():
            summary_image.setPixmap(
                summary_pixmap.scaled(88, 88, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )
        summary_layout.addWidget(summary_image)

        title_box = QVBoxLayout()
        title = QLabel("环境实时监测")
        title.setStyleSheet(
            f"color:{TEXT}; font-size:21px; font-weight:900; border:none; background:transparent;"
        )
        self.updated_label = QLabel("等待环境传感器数据")
        self.updated_label.setStyleSheet(
            f"color:{MUTED}; font-size:12px; border:none; background:transparent;"
        )
        title_box.addWidget(title)
        title_box.addWidget(self.updated_label)
        summary_layout.addLayout(title_box, stretch=1)

        self.temperature_summary = QLabel("-- ℃")
        self.temperature_summary.setStyleSheet(
            f"color:{CYAN}; font-size:29px; font-weight:900; border:none; background:transparent;"
        )
        self.humidity_summary = QLabel("-- %")
        self.humidity_summary.setStyleSheet(
            f"color:{GREEN}; font-size:29px; font-weight:900; border:none; background:transparent;"
        )
        summary_layout.addWidget(self.temperature_summary)
        summary_layout.addWidget(self.humidity_summary)
        layout.addWidget(summary)

        content = QHBoxLayout()
        content.setSpacing(9)
        metrics_panel = QFrame()
        metrics_panel.setStyleSheet("background:transparent; border:none;")
        metrics_grid = QGridLayout(metrics_panel)
        metrics_grid.setContentsMargins(0, 0, 0, 0)
        metrics_grid.setSpacing(7)
        metric_specs = [
            ("temperature", "温度", "℃", CYAN),
            ("humidity", "湿度", "%", GREEN),
            ("formaldehyde", "甲醛", "mg/m³", YELLOW),
            ("pm25", "PM2.5", "μg/m³", PURPLE),
            ("co2", "CO₂", "ppm", ORANGE),
            ("voc", "VOC", "mg/m³", GREEN),
        ]
        self.metric_cards = {}
        for index, (key, title_text, unit, accent) in enumerate(metric_specs):
            card = EnvironmentMetricCard(title_text, unit, accent)
            self.metric_cards[key] = card
            metrics_grid.addWidget(card, index // 3, index % 3)
        content.addWidget(metrics_panel, stretch=5)

        history_panel = QFrame()
        history_panel.setStyleSheet(card_style(PURPLE, SURFACE))
        history_layout = QVBoxLayout(history_panel)
        history_layout.setContentsMargins(10, 8, 10, 8)
        history_title = QLabel("环境历史记录")
        history_title.setStyleSheet(
            f"color:{TEXT}; font-size:17px; font-weight:900; border:none; background:transparent;"
        )
        history_subtitle = QLabel("温度与湿度变化趋势 · 本机保存")
        history_subtitle.setStyleSheet(
            f"color:{MUTED}; font-size:11px; border:none; background:transparent;"
        )
        self.trend = EnvironmentTrendWidget()
        self.trend.set_history(self.history)
        history_layout.addWidget(history_title)
        history_layout.addWidget(history_subtitle)
        history_layout.addWidget(self.trend, stretch=1)
        content.addWidget(history_panel, stretch=4)
        layout.addLayout(content, stretch=1)

        if self.history:
            self._apply_values(self.history[-1], record=False)

    def update_environment(self, data):
        formaldehyde = self._to_mg(
            self._first(data, "甲醛", "formaldehyde", "hcho")
        )
        voc = self._to_mg(self._first(data, "VOC", "TVOC", "voc", "tvoc"))
        values = {
            "temperature": self._first(data, "温度", "temperature", "temp"),
            "humidity": self._first(data, "湿度", "humidity", "humi"),
            "formaldehyde": formaldehyde,
            "pm25": self._first(data, "PM2.5", "pm25", "pm2_5"),
            "co2": self._first(data, "CO2", "co2", "二氧化碳"),
            "voc": voc,
        }
        self._apply_values(values, record=True)

    def _apply_values(self, values, record):
        for key, card in self.metric_cards.items():
            card.set_value(values.get(key))
        temperature = values.get("temperature")
        humidity = values.get("humidity")
        self.temperature_summary.setText(
            f"{temperature if temperature is not None else '--'} ℃"
        )
        self.humidity_summary.setText(
            f"{humidity if humidity is not None else '--'} %"
        )
        self.updated_label.setText(f"更新 {time.strftime('%H:%M:%S')}")

        if record and (temperature is not None or humidity is not None):
            now = time.time()
            if now - self.last_history_time >= 60:
                sample = dict(values)
                sample["timestamp"] = now
                self.history.append(sample)
                self.history = self.history[-1440:]
                self.last_history_time = now
                self.trend.set_history(self.history)
                self._save_history()

    def update_status(self, key, value):
        if key != "air":
            return
        alert = any(flag in value for flag in ("异常", "警告", "危险"))
        self.updated_label.setText(value)
        self.updated_label.setStyleSheet(
            f"color:{RED if alert else GREEN}; font-size:12px; font-weight:800; "
            "border:none; background:transparent;"
        )

    @staticmethod
    def _first(data, *keys):
        for key in keys:
            if key in data and data[key] not in (None, ""):
                return data[key]
        return None

    @staticmethod
    def _to_mg(value):
        try:
            number = float(value)
            return round(number / 1000.0, 3) if number > 2 else number
        except (TypeError, ValueError):
            return value

    def _load_history(self):
        try:
            with open(self.HISTORY_FILE, "r", encoding="utf-8") as stream:
                data = json.load(stream)
            return data if isinstance(data, list) else []
        except (OSError, ValueError, TypeError):
            return []

    def _save_history(self):
        try:
            directory = os.path.dirname(self.HISTORY_FILE)
            os.makedirs(directory, exist_ok=True)
            temporary = self.HISTORY_FILE + ".tmp"
            with open(temporary, "w", encoding="utf-8") as stream:
                json.dump(self.history, stream, ensure_ascii=False)
            os.replace(temporary, self.HISTORY_FILE)
        except OSError:
            pass


class FamilyStatusPage(QWidget):
    HISTORY_FILE = os.path.expanduser(
        "~/.config/wheeltec_ui_dashboard/environment_history.json"
    )
    METRICS = [
        ("temperature", "温度", "℃", CYAN),
        ("humidity", "湿度", "%", GREEN),
        ("formaldehyde", "甲醛", "mg/m³", YELLOW),
        ("pm25", "PM2.5", "μg/m³", PURPLE),
        ("co2", "CO₂", "ppm", ORANGE),
        ("voc", "VOC", "mg/m³", GREEN),
    ]

    def __init__(self, main_window):
        super().__init__()
        self.history = [
            sample
            for sample in self._load_history()
            if not self._is_legacy_placeholder(sample)
        ]
        self.main_window = main_window
        self.last_history_time = (
            float(self.history[-1].get("timestamp", 0)) if self.history else 0.0
        )
        self.current_values = {}
        self.selected_metric = "temperature"
        self.range_hours = 24
        self.last_live_update = 0.0

        root = QVBoxLayout(self)
        root.setContentsMargins(12, 8, 12, 8)
        root.setSpacing(7)
        root.addWidget(page_header(main_window, "家庭环境", "实时数据与历史趋势", CYAN))

        self.scroll = TouchScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.scroll.setFrameShape(QFrame.NoFrame)
        self.scroll.setStyleSheet(f"""
            QScrollArea {{ background:transparent; border:none; }}
            QScrollBar:vertical {{
                background:{BG};
                width:12px;
                margin:2px 0;
                border:none;
            }}
            QScrollBar::handle:vertical {{
                background:{CYAN};
                min-height:52px;
                border-radius:5px;
            }}
            QScrollBar::add-line:vertical,
            QScrollBar::sub-line:vertical {{ height:0; }}
            QScrollBar::add-page:vertical,
            QScrollBar::sub-page:vertical {{ background:transparent; }}
        """)
        self.scroll.viewport().setAttribute(Qt.WA_AcceptTouchEvents, True)
        QScroller.grabGesture(
            self.scroll.viewport(), QScroller.LeftMouseButtonGesture
        )
        body = QWidget()
        body.setStyleSheet(f"background:{BG};")
        body.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)
        content = QVBoxLayout(body)
        content.setContentsMargins(0, 0, 0, 12)
        content.setSpacing(10)

        hero = QFrame()
        hero.setStyleSheet(card_style(GREEN, SURFACE))
        hero_layout = QVBoxLayout(hero)
        hero_layout.setContentsMargins(0, 0, 0, 0)
        hero_layout.setSpacing(0)
        hero_image = CoverImageLabel("safety_alert.png")
        hero_image.setFixedHeight(150)
        hero_layout.addWidget(hero_image)
        caption = QFrame()
        caption.setStyleSheet("background:#0b1220; border:none;")
        caption_layout = QHBoxLayout(caption)
        caption_layout.setContentsMargins(14, 9, 14, 9)
        caption_text = QVBoxLayout()
        caption_title = QLabel("室内空气概览")
        caption_title.setStyleSheet(
            f"color:{TEXT}; font-size:19px; font-weight:900; border:none; background:transparent;"
        )
        caption_subtitle = QLabel("传感器在线时采样 · 历史记录保存在本机")
        caption_subtitle.setStyleSheet(
            f"color:{MUTED}; font-size:12px; border:none; background:transparent;"
        )
        caption_text.addWidget(caption_title)
        caption_text.addWidget(caption_subtitle)
        self.environment_badge = QLabel("等待传感器")
        self.environment_badge.setAlignment(Qt.AlignCenter)
        self.environment_badge.setFixedSize(76, 32)
        self.environment_badge.setStyleSheet(
            f"color:{GREEN}; background:#10251f; border:1px solid {GREEN}; border-radius:7px; font-weight:900;"
        )
        caption_layout.addLayout(caption_text, stretch=1)
        refresh_button = QPushButton("刷新")
        refresh_button.setIcon(self.style().standardIcon(QStyle.SP_BrowserReload))
        refresh_button.setFixedSize(92, 34)
        refresh_button.setStyleSheet(button_style(CYAN))
        refresh_button.clicked.connect(self._request_refresh)
        caption_layout.addWidget(refresh_button)
        caption_layout.addWidget(self.environment_badge)
        hero_layout.addWidget(caption)
        content.addWidget(hero)

        metrics_grid = QGridLayout()
        metrics_grid.setSpacing(8)
        self.metric_cards = {}
        for index, (key, title, unit, accent) in enumerate(self.METRICS):
            card = EnvironmentMetricCard(title, unit, accent)
            card.setFixedHeight(68)
            self.metric_cards[key] = card
            metrics_grid.addWidget(card, index // 2, index % 2)
        content.addLayout(metrics_grid)

        history_header = QHBoxLayout()
        history_title = QLabel("历史趋势")
        history_title.setStyleSheet(
            f"color:{TEXT}; font-size:19px; font-weight:900;"
        )
        history_header.addWidget(history_title)
        history_header.addStretch()
        self.range_buttons = {}
        for hours, label in ((24, "1天"), (168, "7天"), (720, "30天")):
            button = QPushButton(label)
            button.setFixedSize(58, 32)
            button.clicked.connect(
                lambda checked=False, value=hours: self.set_range(value)
            )
            self.range_buttons[hours] = button
            history_header.addWidget(button)
        content.addLayout(history_header)

        metric_row = QHBoxLayout()
        metric_row.setSpacing(6)
        self.metric_buttons = {}
        for key, label, unit, accent in self.METRICS:
            button = QPushButton(label)
            button.setMinimumHeight(34)
            button.clicked.connect(
                lambda checked=False, metric=key: self.select_metric(metric)
            )
            self.metric_buttons[key] = button
            metric_row.addWidget(button)
        content.addLayout(metric_row)

        self.trend = EnvironmentTrendWidget()
        self.trend.setFixedHeight(210)
        trend_frame = QFrame()
        trend_frame.setStyleSheet(card_style(BORDER, SURFACE))
        trend_layout = QVBoxLayout(trend_frame)
        trend_layout.setContentsMargins(8, 8, 8, 8)
        trend_layout.addWidget(self.trend)
        content.addWidget(trend_frame)
        self.sample_count_label = QLabel()
        self.sample_count_label.setStyleSheet(f"color:{MUTED}; font-size:12px;")
        content.addWidget(self.sample_count_label)

        recent_title = QLabel("最近记录")
        recent_title.setStyleSheet(
            f"color:{TEXT}; font-size:19px; font-weight:900;"
        )
        content.addWidget(recent_title)
        self.recent_layout = QVBoxLayout()
        self.recent_layout.setSpacing(7)
        content.addLayout(self.recent_layout)

        self.scroll.setWidget(body)
        root.addWidget(self.scroll, stretch=1)

        self.set_range(24)
        self.select_metric("temperature")
        self._refresh_recent_records()
        self.sensor_timer = QTimer(self)
        self.sensor_timer.timeout.connect(self._check_sensor_online)
        self.sensor_timer.start(2000)

    def showEvent(self, event):
        super().showEvent(event)
        QTimer.singleShot(0, lambda: self.scroll.verticalScrollBar().setValue(0))

    def update_environment(self, data):
        values = {
            "temperature": self._first(data, "温度", "temperature", "temp"),
            "humidity": self._first(data, "湿度", "humidity", "humi"),
            "formaldehyde": self._to_mg(
                self._first(data, "甲醛", "formaldehyde", "hcho")
            ),
            "pm25": self._first(data, "PM2.5", "pm25", "pm2_5"),
            "co2": self._first(data, "CO2", "co2", "二氧化碳"),
            "voc": self._to_mg(
                self._first(data, "VOC", "TVOC", "voc", "tvoc")
            ),
        }
        self.last_live_update = time.time()
        self._apply_values(values, record=True)

    def _apply_values(self, values, record):
        self.current_values.update(values)
        for key, card in self.metric_cards.items():
            card.set_value(self.current_values.get(key))
        self.environment_badge.setText("监测中")
        self.environment_badge.setStyleSheet(
            f"color:{GREEN}; background:#10251f; border:1px solid {GREEN}; "
            "border-radius:7px; font-weight:900;"
        )
        if record and (
            self.current_values.get("temperature") is not None
            or self.current_values.get("humidity") is not None
        ):
            now = time.time()
            if now - self.last_history_time >= 60:
                sample = dict(self.current_values)
                sample["timestamp"] = now
                self.history.append(sample)
                self.history = self.history[-1440:]
                self.last_history_time = now
                self._save_history()
                self._refresh_trend()
                self._refresh_recent_records()

    def set_range(self, hours):
        self.range_hours = hours
        for value, button in self.range_buttons.items():
            selected = value == hours
            button.setStyleSheet(
                button_style(CYAN if selected else MUTED)
                + (f"QPushButton {{ background:#123047; color:{CYAN}; }}" if selected else "")
            )
        self._refresh_trend()

    def select_metric(self, key):
        self.selected_metric = key
        for metric_key, label, unit, accent in self.METRICS:
            selected = metric_key == key
            self.metric_buttons[metric_key].setStyleSheet(
                button_style(accent if selected else MUTED)
                + (f"QPushButton {{ background:#172b3b; color:{accent}; }}" if selected else "")
            )
            if selected:
                self.trend.set_metric(metric_key, label, accent)
        self._refresh_trend()
        self._refresh_recent_records()

    def _request_refresh(self):
        self.environment_badge.setText("刷新中")
        self.environment_badge.setStyleSheet(
            f"color:{CYAN}; background:#102535; border:1px solid {CYAN}; "
            "border-radius:7px; font-weight:900;"
        )
        self.main_window.ros_thread.request_air_refresh()

    def _check_sensor_online(self):
        if self.last_live_update and time.time() - self.last_live_update < 8:
            return
        self.environment_badge.setText("传感器离线")
        self.environment_badge.setStyleSheet(
            f"color:{ORANGE}; background:#2a1d10; border:1px solid {ORANGE}; "
            "border-radius:7px; font-weight:900;"
        )
        for card in self.metric_cards.values():
            card.set_value(None)

    def _refresh_trend(self):
        if not hasattr(self, "trend"):
            return
        cutoff = time.time() - self.range_hours * 3600
        filtered = [
            sample
            for sample in self.history
            if float(sample.get("timestamp", 0)) >= cutoff
        ]
        if not filtered:
            filtered = self.history
        self.trend.set_history(filtered)
        self.sample_count_label.setText(
            f"本机已保存 {len(self.history)} 次环境采样"
            if self.history
            else "等待环境数据，收到采样后会自动生成曲线"
        )

    def _refresh_recent_records(self):
        while self.recent_layout.count():
            item = self.recent_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        if not self.history:
            empty = QLabel("暂无历史记录")
            empty.setAlignment(Qt.AlignCenter)
            empty.setStyleSheet(f"color:{MUTED}; padding:22px;")
            self.recent_layout.addWidget(empty)
            return
        for sample in reversed(self.history[-12:]):
            timestamp = float(sample.get("timestamp", 0))
            when = time.strftime("%m-%d %H:%M", time.localtime(timestamp))
            row = QFrame()
            row.setStyleSheet(card_style(BORDER, SURFACE))
            row_layout = QHBoxLayout(row)
            row_layout.setContentsMargins(12, 9, 12, 9)
            time_label = QLabel(when)
            time_label.setFixedWidth(100)
            time_label.setStyleSheet(f"color:{MUTED};")
            metric = next(
                spec for spec in self.METRICS if spec[0] == self.selected_metric
            )
            key, label, unit, accent = metric
            values = QLabel(label)
            values.setStyleSheet(f"color:{TEXT}; font-weight:800;")
            selected_value = QLabel(f"{sample.get(key, '--')} {unit}".strip())
            selected_value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            selected_value.setStyleSheet(f"color:{accent}; font-weight:800;")
            row_layout.addWidget(time_label)
            row_layout.addWidget(values, stretch=1)
            row_layout.addWidget(selected_value)
            self.recent_layout.addWidget(row)

    def update_status(self, key, value):
        if key != "air":
            return
        if "正在请求" in value:
            self.environment_badge.setText("刷新中")
            self.environment_badge.setStyleSheet(
                f"color:{CYAN}; background:#102535; border:1px solid {CYAN}; "
                "border-radius:7px; font-weight:900;"
            )
            return
        alert = any(flag in value for flag in ("异常", "警告", "危险"))
        self.environment_badge.setText("需关注" if alert else "监测中")
        self.environment_badge.setStyleSheet(
            f"color:{RED if alert else GREEN}; background:{'#2a1119' if alert else '#10251f'}; "
            f"border:1px solid {RED if alert else GREEN}; border-radius:7px; font-weight:900;"
        )

    @staticmethod
    def _first(data, *keys):
        for key in keys:
            if key in data and data[key] not in (None, ""):
                return data[key]
        return None

    @staticmethod
    def _to_mg(value):
        try:
            number = float(value)
            return round(number / 1000.0, 3) if number > 2 else number
        except (TypeError, ValueError):
            return value

    def _load_history(self):
        try:
            with open(self.HISTORY_FILE, "r", encoding="utf-8") as stream:
                data = json.load(stream)
            return data if isinstance(data, list) else []
        except (OSError, ValueError, TypeError):
            return []

    @staticmethod
    def _is_legacy_placeholder(sample):
        try:
            return (
                abs(float(sample.get("temperature", 0)) - 26.5) < 0.001
                and abs(float(sample.get("humidity", 0)) - 54.2) < 0.001
                and abs(float(sample.get("formaldehyde", 0)) - 0.03) < 0.001
                and abs(float(sample.get("pm25", 0)) - 18.0) < 0.001
                and abs(float(sample.get("co2", 0)) - 520.0) < 0.001
                and abs(float(sample.get("voc", 0)) - 0.18) < 0.001
            )
        except (TypeError, ValueError):
            return False

    def _save_history(self):
        try:
            directory = os.path.dirname(self.HISTORY_FILE)
            os.makedirs(directory, exist_ok=True)
            temporary = self.HISTORY_FILE + ".tmp"
            with open(temporary, "w", encoding="utf-8") as stream:
                json.dump(self.history, stream, ensure_ascii=False)
            os.replace(temporary, self.HISTORY_FILE)
        except OSError:
            pass


class QuickActionsPage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(9)
        layout.addWidget(page_header(main_window, "快捷操作", "常用家庭任务一键执行", ORANGE))

        actions = [
            ("客厅巡查", "定点巡查任务", "indoor_patrol.png", lambda: main_window.ros_thread.send_goal(1.20, 0.80), GREEN),
            ("返回守护位", "充电 / 待命点", "indoor_map.png", lambda: main_window.ros_thread.send_goal(0.7325, 0.0205, 0.0153, 0.9999), CYAN),
            ("打开客厅灯", "家庭照明设备", "smart_home.png", lambda: main_window.ros_thread.send_esp32_cmd("LIGHT_ON"), GREEN),
            ("开启室内通风", "改善空气流通", "smart_home.png", lambda: main_window.ros_thread.send_esp32_cmd("FAN_ON"), CYAN),
            ("语音陪伴", "儿童互动看护", "child_care.png", main_window.ros_thread.send_voice_trigger, PURPLE),
            ("查找家庭物品", "水瓶 / 药盒", "life_assist.png", lambda: main_window.ros_thread.send_arm_cmd("SCAN"), PURPLE),
            ("开启声光预警", "家庭异常提醒", "safety_alert.png", lambda: main_window.ros_thread.send_esp32_cmd("ALARM_ON"), ORANGE),
            ("立即停止", "底盘 / 机械臂 / 设备", "emergency_stop.png", main_window.ros_thread.emergency_stop, RED),
        ]
        grid = QGridLayout()
        grid.setSpacing(9)
        for i, (label, description, image_name, callback, accent) in enumerate(actions):
            button = ServiceImageTile(
                label,
                description,
                image_name,
                accent,
                danger=(accent == RED),
            )
            button.clicked.connect(callback)
            grid.addWidget(button, i // 2, i % 2)
        for row in range(4):
            grid.setRowStretch(row, 1)
        layout.addLayout(grid, stretch=1)


class CpuPage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(9)
        layout.addWidget(page_header(main_window, "设备运行状态", "处理器 / 内存 / 家庭守护服务", CYAN))

        cards = QHBoxLayout()
        self.cpu_total = StatCard("CPU 总占用", "--", "%", CYAN)
        self.mem_card = StatCard("内存占用", "--", "%", GREEN)
        self.temp_card = StatCard("守护服务", "稳定", "巡查与看护在线", PURPLE)
        cards.addWidget(self.cpu_total)
        cards.addWidget(self.mem_card)
        cards.addWidget(self.temp_card)
        layout.addLayout(cards)

        self.core_count = min(psutil.cpu_count() or 4, 8)
        self.bars = []
        bars_frame = QFrame()
        bars_frame.setStyleSheet(card_style(CYAN))
        bars_layout = QGridLayout(bars_frame)
        bars_layout.setSpacing(12)
        for i in range(self.core_count):
            name = QLabel(f"Core {i}")
            name.setStyleSheet(f"color:{TEXT}; font-size:16px; border:none; background:transparent;")
            bar = QProgressBar()
            bar.setRange(0, 100)
            bar.setTextVisible(True)
            bar.setStyleSheet(f"""
                QProgressBar {{
                    background-color:{BG};
                    border:1px solid {CYAN};
                    border-radius:8px;
                    color:{TEXT};
                    height:24px;
                    text-align:center;
                    font-weight:700;
                }}
                QProgressBar::chunk {{
                    background-color:{CYAN};
                    border-radius:7px;
                }}
            """)
            self.bars.append(bar)
            bars_layout.addWidget(name, i, 0)
            bars_layout.addWidget(bar, i, 1)
        layout.addWidget(bars_frame, stretch=1)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_cpu_data)
        self.timer.start(500)

    def update_cpu_data(self):
        usages = psutil.cpu_percent(interval=None, percpu=True)
        total = psutil.cpu_percent(interval=None)
        mem = psutil.virtual_memory().percent
        self.cpu_total.set_value(f"{total:.1f}")
        self.mem_card.set_value(f"{mem:.1f}")
        for i in range(min(len(usages), self.core_count)):
            self.bars[i].setValue(int(usages[i]))


class NavPage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(9)
        layout.addWidget(page_header(main_window, "室内巡查", "客厅 / 卧室 / 走廊定点巡查", GREEN))

        grid = QGridLayout()
        grid.setSpacing(8)
        targets = [
            ("返回守护位", 0.732491135597229, 0.020493270829319954, 0.01533924171145018, 0.9998823469107342),
            ("客厅巡查点", 1.20, 0.80, 0.0, 1.0),
            ("卧室巡查点", 2.00, 2.00, 0.0, 1.0),
            ("走廊巡查点", -0.80, 1.50, 0.0, 1.0),
        ]
        for i, (name, x, y, oz, ow) in enumerate(targets):
            btn = QPushButton(f"{name}\nX={x:.2f}  Y={y:.2f}")
            btn.setMinimumHeight(64)
            btn.setStyleSheet(button_style(GREEN))
            btn.clicked.connect(lambda checked, tx=x, ty=y, tz=oz, tw=ow: self.main_window.ros_thread.send_goal(tx, ty, tz, tw))
            grid.addWidget(btn, i // 2, i % 2)
        layout.addLayout(grid)

        stop = QPushButton("停止巡查")
        stop.setFixedHeight(44)
        stop.setStyleSheet(button_style(RED, danger=True))
        stop.clicked.connect(self.main_window.ros_thread.emergency_stop)
        layout.addWidget(stop)
        layout.addStretch()


class ArmVisionPage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(9)
        layout.addWidget(page_header(main_window, "生活协助", "识别并抓取水瓶、药盒等家庭物品", PURPLE))

        top = QHBoxLayout()
        self.camera_card = StatCard("视觉状态", "等待画面", "双目 / 深度", CYAN)
        self.target_card = StatCard("协助目标", "等待识别", "水瓶 / 药盒", PURPLE)
        self.arm_card = StatCard("执行状态", "待命", "机械臂", GREEN)
        top.addWidget(self.camera_card)
        top.addWidget(self.target_card)
        top.addWidget(self.arm_card)
        layout.addLayout(top)

        grid = QGridLayout()
        grid.setSpacing(12)
        cmds = [
            ("查找物品", "SCAN"),
            ("抓取水瓶", "GRAB_BOTTLE"),
            ("抓取药盒", "GRAB_BOX"),
            ("夹爪打开", "GRIPPER_OPEN"),
            ("夹爪闭合", "GRIPPER_CLOSE"),
            ("机械臂复位", "RESET"),
            ("递送物品", "DELIVER"),
            ("立即停止", "STOP"),
        ]
        for i, (label, cmd) in enumerate(cmds):
            danger = cmd == "STOP"
            btn = QPushButton(label)
            btn.setMinimumHeight(48)
            btn.setStyleSheet(button_style(RED if danger else PURPLE, danger=danger))
            btn.clicked.connect(lambda checked, c=cmd: self.main_window.ros_thread.send_arm_cmd(c))
            grid.addWidget(btn, i // 4, i % 4)
        layout.addLayout(grid)

        self.log = QTextBrowser()
        self.log.setStyleSheet(f"""
            QTextBrowser {{
                background:{PANEL};
                color:{TEXT};
                border:1px solid {PURPLE};
                border-radius:12px;
                padding:10px;
                font-size:12px;
            }}
        """)
        self.log.setText("生活协助待命...\n识别到物品后，将在这里显示抓取与递送进度。")
        self.log.document().setMaximumBlockCount(80)
        layout.addWidget(self.log, stretch=1)

    def append_log(self, text):
        self.log.append(f"> {text}")
        if "target" in text.lower() or "xyz" in text.lower():
            self.target_card.set_value(text[:28])
        if text:
            self.arm_card.set_value(text[:20])


class Esp32Page(QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(9)
        layout.addWidget(page_header(main_window, "家居联动", "灯光 / 通风 / 声光预警", ORANGE))

        grid = QGridLayout()
        grid.setSpacing(8)
        commands = [
            ("打开客厅灯", "LIGHT_ON", GREEN),
            ("关闭客厅灯", "LIGHT_OFF", MUTED),
            ("开启室内通风", "FAN_ON", CYAN),
            ("关闭室内通风", "FAN_OFF", MUTED),
            ("开启声光预警", "ALARM_ON", ORANGE),
            ("解除声光预警", "ALARM_OFF", MUTED),
            ("检查设备连接", "STATUS", CYAN),
            ("关闭全部设备", "ALL_OFF", RED),
        ]
        for i, (label, cmd, color) in enumerate(commands):
            btn = QPushButton(label)
            btn.setMinimumHeight(58)
            btn.setStyleSheet(button_style(color, danger=color == RED))
            btn.clicked.connect(lambda checked, c=cmd: self.main_window.ros_thread.send_esp32_cmd(c))
            grid.addWidget(btn, i // 4, i % 4)
        layout.addLayout(grid)
        layout.addStretch()


class AirDataPage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(9)
        layout.addWidget(page_header(main_window, "家庭安全预警", "空气质量 / 异常提醒 / 家居联动", YELLOW))

        self.cards = {}
        grid = QGridLayout()
        grid.setSpacing(8)
        self.keys = [
            ("温度", "温度", "℃"),
            ("湿度", "湿度", "%"),
            ("PM2.5", "PM2.5", "ug/m3"),
            ("PM10", "PM10", "ug/m3"),
            ("CO2", "CO2", "ppm"),
            ("VOC", "TVOC/VOC", "ug/m3"),
            ("甲醛", "甲醛", "ug/m3"),
            ("等级", "空气等级", ""),
        ]
        for i, (key, title, unit) in enumerate(self.keys):
            card = StatCard(title, "--", unit, GREEN)
            self.cards[key] = card
            grid.addWidget(card, i // 4, i % 4)
        layout.addLayout(grid)

        self.advice = QLabel("正在等待家庭环境传感器数据...")
        self.advice.setStyleSheet(f"color:{TEXT}; font-size:18px; padding:14px; background:{PANEL}; border:1px solid {GREEN}; border-radius:12px;")
        layout.addWidget(self.advice)
        layout.addStretch()

    def update_data(self, data_dict):
        for k, v in data_dict.items():
            if k in self.cards:
                self.cards[k].set_value(v)

        level = data_dict.get("等级")
        advice = data_dict.get("建议")
        if level == "异常":
            color = RED
        elif level == "一般":
            color = YELLOW
        elif level == "良好":
            color = GREEN
        else:
            co2 = self._number(data_dict.get("CO2"))
            voc = self._number(data_dict.get("VOC"))
            pm25 = self._number(data_dict.get("PM2.5"))
            level, color, advice = self.evaluate_air(co2, voc, pm25)

        reasons = data_dict.get("异常项") or data_dict.get("提醒项") or []
        if reasons:
            advice = f"{advice} | {'；'.join(str(x) for x in reasons)}"

        self.cards["等级"].set_value(level, color)
        self.advice.setText(advice)
        self.advice.setStyleSheet(
            f"color:{TEXT}; font-size:18px; padding:14px; "
            f"background:{PANEL}; border:2px solid {color}; border-radius:12px;"
        )

    def _number(self, value):
        try:
            return float(value)
        except Exception:
            return None

    def evaluate_air(self, co2, voc, pm25):
        if (co2 and co2 > 1200) or (voc and voc > 600) or (pm25 and pm25 > 75):
            return "异常", RED, "空气质量异常：已触发 ESP32 蜂鸣器并发送手机预警。"
        if (co2 and co2 > 800) or (voc and voc > 300) or (pm25 and pm25 > 35):
            return "一般", YELLOW, "空气质量一般：建议开窗通风并继续观察。"
        return "良好", GREEN, "空气质量良好：系统持续监测中。"


class MapPage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(9)
        layout.addWidget(page_header(main_window, "室内巡查地图", "机器人位置 / 家庭空间地图", CYAN))

        self.map_label = QLabel("等待 /map 与 /amcl_pose 数据...")
        self.map_label.setStyleSheet(f"background-color:{PANEL}; color:{CYAN}; border:1px dashed {CYAN}; border-radius:10px; font-size:14px;")
        self.map_label.setAlignment(Qt.AlignCenter)
        self.map_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.map_label, stretch=1)

        self.base_map_pixmap = None
        self.map_info = None
        self.robot_pose = None

    def update_map_data(self, img_data, ox, oy, res):
        h, w, ch = img_data.shape
        qimg = QImage(img_data.tobytes(), w, h, ch * w, QImage.Format_RGB888).copy()
        self.base_map_pixmap = QPixmap.fromImage(qimg)
        self.map_info = (ox, oy, res)
        self.render_map()

    def update_robot_pose(self, x, y):
        self.robot_pose = (x, y)
        self.render_map()

    def render_map(self):
        if not self.base_map_pixmap:
            return
        display_pixmap = self.base_map_pixmap.copy()
        if self.robot_pose and self.map_info:
            ox, oy, res = self.map_info
            rx, ry = self.robot_pose
            px = int((rx - ox) / res)
            py = display_pixmap.height() - int((ry - oy) / res)
            painter = QPainter(display_pixmap)
            painter.setBrush(QBrush(QColor(255, 77, 109)))
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(px - 5, py - 5, 10, 10)
            painter.setPen(QPen(QColor(255, 77, 109), 2))
            painter.drawEllipse(px - 14, py - 14, 28, 28)
            painter.end()
        target_w, target_h = self.map_label.width() - 10, self.map_label.height() - 10
        if target_w > 0 and target_h > 0:
            self.map_label.setPixmap(display_pixmap.scaled(target_w, target_h, Qt.KeepAspectRatio, Qt.SmoothTransformation))


class VoicePage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(9)
        layout.addWidget(page_header(main_window, "家长聊天", "现场说话将实时发送到家长手机", GREEN))

        self.log = QTextBrowser()
        self.log.setText("家长聊天待命...\n点击“开始说话”，识别后的消息会同步到手机。\n")
        self.log.setStyleSheet(f"""
            QTextBrowser {{
                color:{TEXT};
                background:{PANEL};
                border:1px solid {GREEN};
                border-radius:8px;
                padding:8px;
                font-size:12px;
            }}
        """)
        self.log.document().setMaximumBlockCount(80)

        row = QHBoxLayout()
        self.listening = False
        self.mic_btn = QPushButton("开始说话")
        self.mic_btn.setFixedHeight(44)
        self.mic_btn.setStyleSheet(button_style(GREEN))
        self.mic_btn.clicked.connect(self.toggle_listening)
        row.addWidget(self.mic_btn, stretch=2)

        self.sos_btn = QPushButton("紧急求救")
        self.sos_btn.setFixedHeight(44)
        self.sos_btn.setStyleSheet(button_style(RED, danger=True))
        self.sos_btn.clicked.connect(self.send_sos)
        row.addWidget(self.sos_btn, stretch=1)
        layout.addLayout(row)
        layout.addWidget(self.log, stretch=1)

    def toggle_listening(self):
        self.main_window.ros_thread.set_voice_listening(not self.listening)

    def set_listening(self, enabled):
        self.listening = enabled
        self.mic_btn.setText("结束说话" if enabled else "开始说话")
        self.mic_btn.setStyleSheet(button_style(RED if enabled else GREEN))

    def send_sos(self):
        self.main_window.ros_thread.send_emergency_sos()
        self.sos_btn.setText("求救已发送")
        self.sos_btn.setEnabled(False)
        self.log.append("> 紧急求救已发送，家长手机将弹出提醒并发送短信")
        QTimer.singleShot(5000, self.reset_sos_button)

    def reset_sos_button(self):
        self.sos_btn.setText("紧急求救")
        self.sos_btn.setEnabled(True)

    def append_log(self, text):
        if text == "STATUS:ON":
            self.set_listening(True)
            return
        if text == "STATUS:OFF":
            self.set_listening(False)
            return
        if text == "CLEAR_SCREEN":
            self.log.clear()
            return
        self.log.append(f"> {text}")
        scrollbar = self.log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())


class CareRecordsPage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(9)
        layout.addWidget(
            page_header(main_window, "看护记录", "现场成员、机器人与家长的最近消息", PURPLE)
        )

        self.log = QTextBrowser()
        self.log.setText("暂无看护记录。\n开始家长聊天后，消息会显示在这里。")
        self.log.setStyleSheet(f"""
            QTextBrowser {{
                color:{TEXT};
                background:{PANEL};
                border:1px solid {PURPLE};
                border-radius:8px;
                padding:12px;
                font-size:14px;
            }}
        """)
        self.log.document().setMaximumBlockCount(120)
        layout.addWidget(self.log, stretch=1)

        clear = QPushButton("清空本屏记录")
        clear.setFixedHeight(42)
        clear.setStyleSheet(button_style(MUTED))
        clear.clicked.connect(self.log.clear)
        layout.addWidget(clear)

    def append_log(self, text):
        if text.startswith("STATUS:") or text == "CLEAR_SCREEN":
            return
        self.log.append(f"{time.strftime('%H:%M')}  {text}")
        scrollbar = self.log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())


class OSMainStage(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Window)
        self.setWindowTitle("家庭守护机器人")
        screen = QApplication.primaryScreen()
        if screen:
            self.setGeometry(screen.geometry())
        else:
            self.resize(800, 480)
        self.setStyleSheet(f"background-color:{BG}; font-family:'Noto Sans CJK SC','Microsoft YaHei',Arial;")

        self.inactivity_timer = QTimer(self)
        self.inactivity_timer.setSingleShot(True)
        self.inactivity_timer.setInterval(5000)
        self.inactivity_timer.timeout.connect(self.return_to_eyes)

        self.ros_thread = Ros2EngineThread()
        self.ros_thread.start()

        self.stacked_widget = QStackedWidget()
        self.setCentralWidget(self.stacked_widget)

        self.eye_page = EyeHomePage(self)
        self.main_menu = MainMenuPage(self)
        self.cpu_page = CpuPage(self)
        self.nav_page = NavPage(self)
        self.arm_page = ArmVisionPage(self)
        self.esp32_page = Esp32Page(self)
        self.air_page = AirDataPage(self)
        self.map_page = MapPage(self)
        self.voice_page = VoicePage(self)
        self.care_records_page = CareRecordsPage(self)
        self.services_page = GuardianServicesPage(self)
        self.family_status_page = FamilyStatusPage(self)
        self.quick_actions_page = QuickActionsPage(self)
        self.module_pages = {
            "services": self.services_page,
            "status": self.family_status_page,
        }
        self.service_pages = {
            "map": self.map_page,
            "patrol": self.nav_page,
            "safety": self.air_page,
            "smart_home": self.esp32_page,
            "records": self.care_records_page,
            "chat": self.voice_page,
        }

        for page in [
            self.eye_page,
            self.main_menu,
            self.cpu_page,
            self.nav_page,
            self.arm_page,
            self.esp32_page,
            self.air_page,
            self.map_page,
            self.voice_page,
            self.care_records_page,
            self.services_page,
            self.family_status_page,
            self.quick_actions_page,
        ]:
            self.stacked_widget.addWidget(page)

        self.ros_thread.map_signal.connect(self.map_page.update_map_data)
        self.ros_thread.pose_signal.connect(self.map_page.update_robot_pose)
        self.ros_thread.air_data_signal.connect(self.air_page.update_data)
        self.ros_thread.air_data_signal.connect(self.main_menu.update_environment)
        self.ros_thread.air_data_signal.connect(
            self.family_status_page.update_environment
        )
        self.ros_thread.voice_log_signal.connect(self.voice_page.append_log)
        self.ros_thread.voice_log_signal.connect(self.care_records_page.append_log)
        self.ros_thread.status_signal.connect(self.main_menu.update_status)
        self.ros_thread.status_signal.connect(self.family_status_page.update_status)
        self.ros_thread.arm_log_signal.connect(self.arm_page.append_log)

    def open_module(self, module_name):
        page = self.module_pages.get(module_name)
        if page is None:
            return
        self.stacked_widget.setCurrentWidget(page)
        self.inactivity_timer.start()

    def open_service(self, service_name):
        page = self.service_pages.get(service_name)
        if page is None:
            return
        self.stacked_widget.setCurrentWidget(page)
        self.inactivity_timer.start()

    def go_back(self):
        current_page = self.stacked_widget.currentWidget()
        if current_page in self.service_pages.values():
            self.stacked_widget.setCurrentWidget(self.services_page)
        else:
            self.stacked_widget.setCurrentWidget(self.main_menu)
        self.inactivity_timer.start()

    def switch_to_page(self, index):
        self.stacked_widget.setCurrentIndex(index)
        if index == 0:
            self.inactivity_timer.stop()
        else:
            self.inactivity_timer.start()

    def return_to_eyes(self):
        if self.stacked_widget.currentIndex() != 0:
            self.stacked_widget.setCurrentIndex(0)
        self.inactivity_timer.stop()

    def eventFilter(self, watched, event):
        activity_events = (
            QEvent.MouseButtonPress,
            QEvent.KeyPress,
            QEvent.TouchBegin,
            QEvent.Wheel,
        )
        if event.type() in activity_events and self.stacked_widget.currentIndex() != 0:
            self.inactivity_timer.start()
        return super().eventFilter(watched, event)

    def closeEvent(self, event):
        self.ros_thread.stop()
        self.ros_thread.wait()
        super().closeEvent(event)


def main(args=None):
    app = QApplication(sys.argv)
    app.setFont(QFont("Microsoft YaHei", 10))
    window = OSMainStage()
    app.installEventFilter(window)

    signal.signal(signal.SIGINT, lambda _signum, _frame: window.close())
    signal.signal(signal.SIGTERM, lambda _signum, _frame: window.close())
    signal_timer = QTimer()
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start(200)

    window.showFullScreen()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
