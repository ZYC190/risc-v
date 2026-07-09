import sys
import os
import json
import time
import math
import psutil
import numpy as np

import rclpy
from rclpy.node import Node
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
)
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal
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

    def send_arm_cmd(self, cmd):
        self.publish_string(self.arm_cmd_pub, cmd)
        self.arm_log_signal.emit(f"已下发机械臂指令: {cmd}")
        self.status_signal.emit("arm", f"指令: {cmd}")

    def send_voice_trigger(self):
        self.publish_string(self.voice_cmd_pub, "TOGGLE_LISTENING")

    def emergency_stop(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.send_arm_cmd("STOP")
        self.send_esp32_cmd("ALL_OFF")
        self.status_signal.emit("task", "急停：底盘零速度 / 机械臂停止 / 物联关闭")

    def run(self):
        rclpy.spin(self.node)

    def stop(self):
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.quit()


def create_back_button(parent):
    btn = QPushButton("返回")
    btn.setFixedHeight(38)
    btn.setStyleSheet(button_style(MUTED))
    btn.clicked.connect(lambda: parent.switch_to_page(1))
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


class FeatureTile(QPushButton):
    def __init__(self, title, desc, accent=CYAN, danger=False):
        super().__init__(f"{title}\n{desc}")
        self.setMinimumHeight(70)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet(f"""
            QPushButton {{
                background-color:{SURFACE};
                border:1px solid {RED if danger else BORDER};
                border-left:5px solid {RED if danger else accent};
                border-radius:8px;
                color:{TEXT};
                text-align:left;
                padding:8px 12px;
                font-size:15px;
                font-weight:800;
            }}
            QPushButton:hover {{
                background-color:{SURFACE_2};
                border:1px solid {RED if danger else accent};
                border-left:5px solid {RED if danger else accent};
            }}
            QPushButton:pressed {{
                background-color:{RED if danger else accent};
                color:#06101d;
            }}
        """)


class AnimatedEyesWidget(QWidget):
    """Cute animated standby eyes for the robot touchscreen."""

    activated = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.t = 0.0
        self.blink = 1.0
        self.setMinimumSize(1, 1)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(33)

    def tick(self):
        self.t += 0.055
        phase = self.t % 6.0
        if phase > 5.55:
            self.blink = max(0.38, 1.0 - (phase - 5.55) * 3.2)
        elif phase < 0.28:
            self.blink = min(1.0, 0.38 + phase * 2.4)
        else:
            self.blink = 1.0
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.activated.emit()
        super().mousePressEvent(event)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        screen = self.window().screen()
        if screen:
            geo = screen.geometry()
            w = min(w, geo.width())
            h = min(h, geo.height())
        cx, cy = w * 0.50, h * 0.51
        face_w = min(w * 0.76, 760)
        face_h = min(h * 0.62, 350)

        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(20, 36, 58)))
        painter.drawRoundedRect(int(cx - face_w / 2), int(cy - face_h / 2), int(face_w), int(face_h), 70, 70)
        painter.setBrush(QBrush(QColor(9, 18, 32)))
        painter.drawRoundedRect(int(cx - face_w / 2 + 24), int(cy - face_h / 2 + 24), int(face_w - 48), int(face_h - 50), 56, 56)

        eye_w = int(face_w * 0.34)
        eye_h = max(60, int(eye_w * 0.52 * (0.78 + self.blink * 0.22)))
        gap = face_w * 0.21
        look_x = math.sin(self.t * 0.9) * min(18, eye_w * 0.10)
        look_y = math.sin(self.t * 0.55) * min(10, eye_h * 0.10)

        for side in (-1, 1):
            ex = cx + side * gap
            ey = cy - face_h * 0.08
            painter.setBrush(QBrush(QColor(120, 220, 255, 38)))
            painter.drawRoundedRect(int(ex - eye_w / 2 - 8), int(ey - eye_h / 2 - 8), eye_w + 16, eye_h + 16, 58, 58)
            painter.setBrush(QBrush(QColor(232, 248, 255)))
            painter.drawRoundedRect(int(ex - eye_w / 2), int(ey - eye_h / 2), eye_w, eye_h, 58, 58)
            pupil_outer = max(58, int(eye_h * 0.68))
            pupil_inner = max(24, int(pupil_outer * 0.43))
            highlight = max(10, int(pupil_outer * 0.20))
            painter.setBrush(QBrush(QColor(80, 205, 255)))
            painter.drawEllipse(int(ex - pupil_outer / 2 + look_x), int(ey - pupil_outer / 2 + look_y), pupil_outer, pupil_outer)
            painter.setBrush(QBrush(QColor(5, 12, 22)))
            painter.drawEllipse(int(ex - pupil_inner / 2 + look_x), int(ey - pupil_inner / 2 + look_y), pupil_inner, pupil_inner)
            painter.setBrush(QBrush(QColor(255, 255, 255)))
            painter.drawEllipse(int(ex + pupil_inner * 0.15 + look_x), int(ey - pupil_inner * 0.62 + look_y), highlight, highlight)

        painter.setBrush(QBrush(QColor(255, 108, 145, 120)))
        painter.drawEllipse(int(cx - face_w * 0.40), int(cy + face_h * 0.15), 58, 28)
        painter.drawEllipse(int(cx + face_w * 0.33), int(cy + face_h * 0.15), 58, 28)
        painter.setPen(QPen(QColor(57, 242, 166), 5))
        painter.drawArc(int(cx - 58), int(cy + face_h * 0.13), 116, 72, 200 * 16, 140 * 16)


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
        btn = QPushButton("MENU", self)
        btn.setToolTip("进入控制界面")
        btn.setFixedSize(78, 46)
        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: rgba(6,16,29,0.86);
                border: 2px solid {YELLOW};
                border-radius: 12px;
                color: {YELLOW};
                font-size: 15px;
                font-weight: 800;
                padding: 0px;
            }}
            QPushButton:hover {{
                background-color: {YELLOW};
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
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(9)

        header_card = QFrame()
        header_card.setStyleSheet(card_style(BORDER, SURFACE))
        header = QHBoxLayout(header_card)
        header.setContentsMargins(16, 10, 16, 10)
        header.setSpacing(12)

        title_box = QVBoxLayout()
        title_box.setSpacing(1)
        title = QLabel("Robot Control Center")
        title.setStyleSheet(f"color:{TEXT}; font-size:22px; font-weight:900; border:none; background:transparent;")
        subtitle = QLabel("K1 MUSE Pi Pro  |  ROS2 多任务调度  |  MQTT 物联桥接")
        subtitle.setStyleSheet(f"color:{MUTED}; font-size:12px; border:none; background:transparent;")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header.addLayout(title_box, stretch=1)

        self.health_badge = QLabel("系统待命")
        self.health_badge.setAlignment(Qt.AlignCenter)
        self.health_badge.setFixedSize(96, 32)
        self.health_badge.setStyleSheet(f"""
            QLabel {{
                color:{GREEN};
                background:#10251f;
                border:1px solid {GREEN};
                border-radius:8px;
                font-size:13px;
                font-weight:800;
            }}
        """)
        header.addWidget(self.health_badge)

        self.clock = QLabel()
        self.clock.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.clock.setMinimumWidth(100)
        self.clock.setStyleSheet(f"color:{CYAN}; font-size:18px; font-weight:900; border:none; background:transparent;")
        header.addWidget(self.clock)
        root.addWidget(header_card)

        body = QHBoxLayout()
        body.setSpacing(10)

        nav_card = QFrame()
        nav_card.setFixedWidth(386)
        nav_card.setStyleSheet(card_style(BORDER, PANEL))
        nav_wrap = QVBoxLayout(nav_card)
        nav_wrap.setContentsMargins(12, 12, 12, 12)
        nav_wrap.setSpacing(10)

        nav_title = QLabel("功能入口")
        nav_title.setStyleSheet(f"color:{TEXT}; font-size:16px; font-weight:900; border:none; background:transparent;")
        nav_wrap.addWidget(nav_title)

        nav_grid = QGridLayout()
        nav_grid.setSpacing(8)
        buttons = [
            ("算力监控", "CPU / 内存", 2, CYAN),
            ("自主导航", "目标点派发", 3, GREEN),
            ("视觉抓取", "双目 / 机械臂", 4, PURPLE),
            ("物联控制", "ESP32 外设", 5, ORANGE),
            ("空气质量", "7合1传感器", 6, GREEN),
            ("SLAM地图", "定位 / 地图", 7, CYAN),
            ("语音交互", "声源 / API", 8, PURPLE),
            ("急停保护", "执行端停止", -1, RED),
        ]
        for i, (title, desc, page, accent) in enumerate(buttons):
            btn = FeatureTile(title, desc, accent, danger=(page == -1))
            if page == -1:
                btn.clicked.connect(self.main_window.ros_thread.emergency_stop)
            else:
                btn.clicked.connect(lambda checked, idx=page: self.main_window.switch_to_page(idx))
            nav_grid.addWidget(btn, i // 2, i % 2)
        nav_wrap.addLayout(nav_grid, stretch=1)
        body.addWidget(nav_card)

        right = QVBoxLayout()
        right.setSpacing(10)

        overview = QFrame()
        overview.setStyleSheet(card_style(BORDER, PANEL))
        overview_layout = QVBoxLayout(overview)
        overview_layout.setContentsMargins(12, 12, 12, 12)
        overview_layout.setSpacing(10)
        overview_title = QLabel("实时状态")
        overview_title.setStyleSheet(f"color:{TEXT}; font-size:16px; font-weight:900; border:none; background:transparent;")
        overview_layout.addWidget(overview_title)

        self.status_cards = {
            "task": DashboardStatusCard("任务状态", "待命", YELLOW),
            "pose": DashboardStatusCard("机器人位姿", "等待定位", CYAN),
            "air": DashboardStatusCard("空气传感", "等待数据", GREEN),
            "esp32": DashboardStatusCard("ESP32物联", "等待心跳", ORANGE),
            "arm": DashboardStatusCard("机械臂", "待命", PURPLE),
            "voice": DashboardStatusCard("语音交互", "待命", PURPLE),
        }
        status_grid = QGridLayout()
        status_grid.setSpacing(8)
        for i, card in enumerate(self.status_cards.values()):
            status_grid.addWidget(card, i // 3, i % 3)
        overview_layout.addLayout(status_grid)
        right.addWidget(overview, stretch=2)

        quick = QFrame()
        quick.setStyleSheet(card_style(BORDER, PANEL))
        quick_layout = QVBoxLayout(quick)
        quick_layout.setContentsMargins(12, 12, 12, 12)
        quick_layout.setSpacing(8)
        quick_title = QLabel("演示快捷操作")
        quick_title.setStyleSheet(f"color:{TEXT}; font-size:16px; font-weight:900; border:none; background:transparent;")
        quick_layout.addWidget(quick_title)

        quick_row = QGridLayout()
        quick_row.setSpacing(8)
        quick_buttons = [
            ("开灯", lambda: self.main_window.ros_thread.send_esp32_cmd("LIGHT_ON"), GREEN),
            ("关灯", lambda: self.main_window.ros_thread.send_esp32_cmd("LIGHT_OFF"), MUTED),
            ("风扇", lambda: self.main_window.ros_thread.send_esp32_cmd("FAN_ON"), CYAN),
            ("报警", lambda: self.main_window.ros_thread.send_esp32_cmd("ALARM_ON"), ORANGE),
            ("全关", lambda: self.main_window.ros_thread.send_esp32_cmd("ALL_OFF"), RED),
            ("监听", self.main_window.ros_thread.send_voice_trigger, PURPLE),
        ]
        for i, (text, callback, accent) in enumerate(quick_buttons):
            b = QPushButton(text)
            b.setFixedHeight(42)
            b.setMinimumWidth(70)
            b.setStyleSheet(button_style(accent, danger=(accent == RED)))
            b.clicked.connect(callback)
            quick_row.addWidget(b, i // 3, i % 3)
        quick_layout.addLayout(quick_row)

        self.status_line = QLabel("提示：先确认 ROS2 / MQTT / ESP32 在线，再进行导航或机械臂演示。")
        self.status_line.setWordWrap(True)
        self.status_line.setStyleSheet(
            f"color:{MUTED}; font-size:13px; padding:8px; "
            f"background:{SURFACE}; border:1px solid {BORDER}; border-radius:8px;"
        )
        quick_layout.addWidget(self.status_line)
        right.addWidget(quick, stretch=1)

        body.addLayout(right, stretch=1)
        root.addLayout(body, stretch=1)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_clock)
        self.timer.start(1000)
        self.update_clock()

    def update_clock(self):
        self.clock.setText(time.strftime("%H:%M:%S"))

    def update_status(self, key, value):
        if key in self.status_cards:
            accent = GREEN
            if key == "task":
                accent = YELLOW
            elif key == "esp32":
                accent = ORANGE
            elif key in ("arm", "voice"):
                accent = PURPLE
            elif key == "pose":
                accent = CYAN
            if "异常" in value or "急停" in value or "STOP" in value:
                accent = RED
            self.status_cards[key].set_value(value, accent)
            self.status_line.setText(f"{key}: {value}")
            self.health_badge.setText("在线运行")
            self.health_badge.setStyleSheet(f"""
                QLabel {{
                    color:{GREEN};
                    background:#10251f;
                    border:1px solid {GREEN};
                    border-radius:8px;
                    font-size:13px;
                    font-weight:800;
                }}
            """)


class CpuPage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(9)
        layout.addWidget(page_header(main_window, "K1 八核算力监控", "CPU / 内存 / 多任务运行状态", CYAN))

        cards = QHBoxLayout()
        self.cpu_total = StatCard("CPU 总占用", "--", "%", CYAN)
        self.mem_card = StatCard("内存占用", "--", "%", GREEN)
        self.temp_card = StatCard("系统状态", "稳定", "多任务运行", PURPLE)
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
        layout.addWidget(page_header(main_window, "自主导航", "Nav2 目标点派发 / 底盘急停保护", GREEN))

        grid = QGridLayout()
        grid.setSpacing(8)
        targets = [
            ("充电点 / 起始点", 0.732491135597229, 0.020493270829319954, 0.01533924171145018, 0.9998823469107342),
            ("桌边抓取点", 1.20, 0.80, 0.0, 1.0),
            ("巡航点 A", 2.00, 2.00, 0.0, 1.0),
            ("巡航点 B", -0.80, 1.50, 0.0, 1.0),
        ]
        for i, (name, x, y, oz, ow) in enumerate(targets):
            btn = QPushButton(f"{name}\nX={x:.2f}  Y={y:.2f}")
            btn.setMinimumHeight(64)
            btn.setStyleSheet(button_style(GREEN))
            btn.clicked.connect(lambda checked, tx=x, ty=y, tz=oz, tw=ow: self.main_window.ros_thread.send_goal(tx, ty, tz, tw))
            grid.addWidget(btn, i // 2, i % 2)
        layout.addLayout(grid)

        stop = QPushButton("停止导航 / 底盘零速度")
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
        layout.addWidget(page_header(main_window, "视觉抓取", "双目相机识别 / 机械臂动作控制", PURPLE))

        top = QHBoxLayout()
        self.camera_card = StatCard("相机输入", "/camera/color/image_raw", "双目/深度画面", CYAN)
        self.target_card = StatCard("目标坐标", "等待识别", "target_xyz / object position", PURPLE)
        self.arm_card = StatCard("机械臂状态", "待命", "串口动作帧", GREEN)
        top.addWidget(self.camera_card)
        top.addWidget(self.target_card)
        top.addWidget(self.arm_card)
        layout.addLayout(top)

        grid = QGridLayout()
        grid.setSpacing(12)
        cmds = [
            ("扫描目标", "SCAN"),
            ("抓取水瓶", "GRAB_BOTTLE"),
            ("抓取药盒", "GRAB_BOX"),
            ("夹爪打开", "GRIPPER_OPEN"),
            ("夹爪闭合", "GRIPPER_CLOSE"),
            ("机械臂复位", "RESET"),
            ("递送到前方", "DELIVER"),
            ("停止机械臂", "STOP"),
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
        self.log.setText("机械臂日志待命...\n可在抓取脚本中发布 /arm_status 显示识别坐标、IK 结果和动作完成状态。")
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
        layout.addWidget(page_header(main_window, "ESP32 物联控制", "灯光 / 风扇 / 蜂鸣器 / 继电器", ORANGE))

        grid = QGridLayout()
        grid.setSpacing(8)
        commands = [
            ("打开灯光", "LIGHT_ON", GREEN),
            ("关闭灯光", "LIGHT_OFF", RED),
            ("打开风扇", "FAN_ON", GREEN),
            ("关闭风扇", "FAN_OFF", RED),
            ("报警器开启", "ALARM_ON", ORANGE),
            ("报警器关闭", "ALARM_OFF", RED),
            ("查询节点状态", "STATUS", CYAN),
            ("全部关闭", "ALL_OFF", RED),
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
        layout.addWidget(page_header(main_window, "空气质量监测", "七合一传感器 / 手机预警 / ESP32 联动", GREEN))

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

        self.advice = QLabel("等待环境传感器数据...")
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
        layout.addWidget(page_header(main_window, "SLAM 地图", "Cartographer 建图 / AMCL 位姿显示", CYAN))

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
        layout.addWidget(page_header(main_window, "语音交互", "声源定位 / API 对话 / 语音监听", PURPLE))

        self.log = QTextBrowser()
        self.log.setText("语音交互日志待命...\n")
        self.log.setStyleSheet(f"""
            QTextBrowser {{
                color:{TEXT};
                background:{PANEL};
                border:1px solid {PURPLE};
                border-radius:12px;
                padding:8px;
                font-size:12px;
            }}
        """)
        self.log.document().setMaximumBlockCount(80)
        layout.addWidget(self.log, stretch=1)

        row = QHBoxLayout()
        self.mic_btn = QPushButton("切换语音监听")
        self.mic_btn.setStyleSheet(button_style(PURPLE))
        self.mic_btn.clicked.connect(self.main_window.ros_thread.send_voice_trigger)
        row.addWidget(self.mic_btn)

        clear = QPushButton("清空日志")
        clear.setStyleSheet(button_style(MUTED))
        clear.clicked.connect(self.log.clear)
        row.addWidget(clear)
        layout.addLayout(row)

    def append_log(self, text):
        if text == "CLEAR_SCREEN":
            self.log.clear()
            return
        self.log.append(f"> {text}")
        scrollbar = self.log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())


class OSMainStage(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Window)
        self.setWindowTitle("K1 MUSE Pi Pro Robot Dashboard")
        screen = QApplication.primaryScreen()
        if screen:
            self.setGeometry(screen.geometry())
        else:
            self.resize(800, 480)
        self.setStyleSheet(f"background-color:{BG}; font-family:'Noto Sans CJK SC','Microsoft YaHei',Arial;")

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
        ]:
            self.stacked_widget.addWidget(page)

        self.ros_thread.map_signal.connect(self.map_page.update_map_data)
        self.ros_thread.pose_signal.connect(self.map_page.update_robot_pose)
        self.ros_thread.air_data_signal.connect(self.air_page.update_data)
        self.ros_thread.voice_log_signal.connect(self.voice_page.append_log)
        self.ros_thread.status_signal.connect(self.main_menu.update_status)
        self.ros_thread.arm_log_signal.connect(self.arm_page.append_log)

    def switch_to_page(self, index):
        self.stacked_widget.setCurrentIndex(index)

    def closeEvent(self, event):
        self.ros_thread.stop()
        self.ros_thread.wait()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setFont(QFont("Microsoft YaHei", 10))
    window = OSMainStage()
    window.showFullScreen()
    sys.exit(app.exec_())
