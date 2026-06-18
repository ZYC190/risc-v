import sys
import json
import psutil
import numpy as np

# --- ROS2 相关库 ---
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# --- PyQt5 相关库 ---
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QPushButton, QGroupBox, QSizePolicy, QStackedWidget, 
                             QGridLayout, QTextBrowser, QFrame)
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QImage, QPixmap, QPainter, QBrush, QColor, QPen
import pyqtgraph as pg

# ==========================================
# 模块 A：后台 ROS2 通信神经中枢
# ==========================================
class Ros2EngineThread(QThread):
    map_signal = pyqtSignal(object, float, float, float)
    pose_signal = pyqtSignal(float, float) 
    air_data_signal = pyqtSignal(dict)
    voice_log_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('gui_commander_node')
        
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.map_sub = self.node.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.pose_sub = self.node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.air_sub = self.node.create_subscription(String, '/air_sensor_data', self.air_callback, 10)
        self.voice_sub = self.node.create_subscription(String, '/voice_log', self.voice_callback, 10)

        self.goal_pub = self.node.create_publisher(PoseStamped, '/goal_pose', 10)
        self.esp32_pub = self.node.create_publisher(String, '/esp32_cmd', 10)
        self.voice_cmd_pub = self.node.create_publisher(String, '/voice_trigger', 10)

    def map_callback(self, msg):
        width, height = msg.info.width, msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        img_data = np.full((height, width, 3), 10, dtype=np.uint8) 
        img_data[data == -1] = [5, 10, 15]     
        img_data[data == 0] = [20, 30, 45]     
        img_data[data >= 50] = [0, 240, 255]   
        img_data = np.flipud(img_data)
        self.map_signal.emit(img_data, ox, oy, res)

    def pose_callback(self, msg):
        self.pose_signal.emit(msg.pose.pose.position.x, msg.pose.pose.position.y)

    def air_callback(self, msg):
        try:
            data_dict = json.loads(msg.data)
            self.air_data_signal.emit(data_dict)
        except Exception as e:
            pass

    def voice_callback(self, msg):
        self.voice_log_signal.emit(msg.data)

    def send_goal(self, target_x, target_y, oz=0.0, ow=1.0):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = float(target_x)
        goal_msg.pose.position.y = float(target_y)
        goal_msg.pose.orientation.z = float(oz)
        goal_msg.pose.orientation.w = float(ow)
        self.goal_pub.publish(goal_msg)

    def send_esp32_cmd(self, cmd):
        msg = String()
        msg.data = cmd
        self.esp32_pub.publish(msg)

    def send_voice_trigger(self):
        msg = String()
        msg.data = "TOGGLE_LISTENING"
        self.voice_cmd_pub.publish(msg)

    def run(self):
        rclpy.spin(self.node)

    def stop(self):
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.quit()

# ==========================================
# 模块 B：前端子页面组件 & 美化UI
# ==========================================
def create_back_button(parent):
    btn = QPushButton("◄ 返回主控矩阵")
    btn.setStyleSheet("""
        QPushButton { 
            background-color: rgba(255, 0, 60, 0.1); 
            border: 1px solid #FF003C; 
            border-radius: 4px;
            color: #FF003C; 
            padding: 10px 20px; 
            font-size: 16px;
            font-weight: bold; 
            font-family: Consolas;
            text-align: left;
        }
        QPushButton:hover { 
            background-color: #FF003C; 
            color: #FFFFFF; 
        }
    """)
    btn.clicked.connect(lambda: parent.switch_to_page(0))
    return btn

class CpuPage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(create_back_button(main_window))
        
        self.plot_widget = pg.PlotWidget(title="< CORE PERFORMANCE MATRIX >")
        self.plot_widget.setYRange(0, 100)
        self.plot_widget.showGrid(x=True, y=True, alpha=0.4)
        
        font = QFont("Consolas", 10)
        self.plot_widget.getAxis("bottom").setTickFont(font)
        self.plot_widget.getAxis("left").setTickFont(font)
        self.plot_widget.getAxis('left').setPen('#00F0FF')
        self.plot_widget.getAxis('bottom').setPen('#00F0FF')
        self.plot_widget.setBackground('#0A0E17') 
        
        self.legend = self.plot_widget.addLegend(offset=(20, 20))
        self.legend.setBrush(pg.mkBrush(color=(10, 14, 23, 200)))
        layout.addWidget(self.plot_widget) 

        self.core_count = psutil.cpu_count() or 4
        if self.core_count > 8: self.core_count = 8 

        self.num_points = 50 
        self.cpu_data = np.zeros((self.core_count, self.num_points))
        self.curves = []
        neon_colors = [(0, 240, 255), (255, 0, 255), (0, 255, 128), (255, 255, 0), 
                       (255, 80, 80), (180, 0, 255), (255, 165, 0), (200, 200, 255)]
        
        for i in range(self.core_count):
            curve = self.plot_widget.plot(self.cpu_data[i], pen=pg.mkPen(color=neon_colors[i], width=2.5), name=f" Core {i} ")
            self.curves.append(curve)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_cpu_data)
        self.timer.start(500)

    def update_cpu_data(self):
        usages = psutil.cpu_percent(interval=None, percpu=True)
        for i in range(min(len(usages), self.core_count)):
            self.cpu_data[i] = np.roll(self.cpu_data[i], -1)
            self.cpu_data[i][-1] = usages[i]
            self.curves[i].setData(self.cpu_data[i])

class NavPage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        layout = QVBoxLayout(self)
        layout.addWidget(create_back_button(main_window))
        
        lbl = QLabel("❖ 战术目标覆写 (NAV_OVERRIDE) ❖")
        lbl.setStyleSheet("color: #00F0FF; font-size: 26px; font-weight: bold; letter-spacing: 2px; margin: 20px 0;")
        lbl.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl)

        btn_style = """
            QPushButton { 
                background-color: rgba(0, 240, 255, 0.05); 
                border: 2px solid #00F0FF; 
                border-radius: 8px;
                color: #00F0FF; 
                padding: 30px; 
                font-size: 22px; 
                font-weight: bold;
                letter-spacing: 3px;
            } 
            QPushButton:hover { background-color: #00F0FF; color: #000; }
        """
        btn_a = QPushButton("⌖ 目标 A 点 : 充电补给站")
        btn_a.setStyleSheet(btn_style)
        btn_a.clicked.connect(lambda: self.main_window.ros_thread.send_goal(0.732491135597229, 0.020493270829319954, 0.01533924171145018, 0.9998823469107342))
        layout.addWidget(btn_a)

        btn_b = QPushButton("⌖ 目标 B 点 : 巡逻防御区")
        btn_b.setStyleSheet(btn_style)
        btn_b.clicked.connect(lambda: self.main_window.ros_thread.send_goal(2.0, 2.0))
        layout.addWidget(btn_b)
        layout.addStretch()

class Esp32Page(QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        layout = QVBoxLayout(self)
        layout.addWidget(create_back_button(main_window))
        
        lbl = QLabel("❖ 外部硬件节点直连 (ESP32 IOT) ❖")
        lbl.setStyleSheet("color: #00F0FF; font-size: 26px; font-weight: bold; letter-spacing: 2px; margin: 20px 0;")
        lbl.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl)

        btn_on = QPushButton("💡 强行点亮矩阵 (FORCE ON)")
        btn_on.setStyleSheet("""
            QPushButton { background-color: rgba(0, 255, 128, 0.05); border: 2px solid #00FF80; border-radius: 8px; color: #00FF80; padding: 40px; font-size: 24px; font-weight: bold;} 
            QPushButton:hover { background-color: #00FF80; color: #000; }
        """)
        btn_on.clicked.connect(lambda: self.main_window.ros_thread.send_esp32_cmd("ON"))
        
        btn_off = QPushButton("🌑 强行切断能源 (FORCE OFF)")
        btn_off.setStyleSheet("""
            QPushButton { background-color: rgba(255, 0, 60, 0.05); border: 2px solid #FF003C; border-radius: 8px; color: #FF003C; padding: 40px; font-size: 24px; font-weight: bold;} 
            QPushButton:hover { background-color: #FF003C; color: #000; }
        """)
        btn_off.clicked.connect(lambda: self.main_window.ros_thread.send_esp32_cmd("OFF"))

        layout.addWidget(btn_on)
        layout.addWidget(btn_off)
        layout.addStretch()

class AirDataPage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(create_back_button(main_window))
        
        lbl = QLabel("❖ 七合一环境感知雷达 ❖")
        lbl.setStyleSheet("color: #00F0FF; font-size: 24px; font-weight: bold; margin-bottom: 5px;")
        lbl.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl)

        self.labels = {}
        grid = QGridLayout()
        grid.setSpacing(10)
        
        keys = ["温度", "湿度", "PM2.5", "PM10", "CO2", "VOC", "甲醛"]
        names = ["温度 (℃)", "湿度 (%)", "PM2.5 (ug/m³)", "PM10 (ug/m³)", "CO2 (ppm)", "VOC (ug/m³)", "甲醛 (ug/m³)"]
        
        # 💡 横屏终极适配：改为 4 列布局，彻底解决挤出屏幕问题！
        for i, key in enumerate(keys):
            frame = QFrame()
            frame.setStyleSheet("""
                QFrame {
                    background-color: rgba(0, 240, 255, 0.03);
                    border: 1px solid rgba(0, 240, 255, 0.3);
                    border-radius: 8px;
                }
            """)
            flayout = QVBoxLayout(frame)
            
            title = QLabel(names[i])
            title.setStyleSheet("color: #66AABB; font-size: 13px; border: none; background: transparent;")
            title.setAlignment(Qt.AlignCenter)
            
            val = QLabel("--")
            val.setStyleSheet("color: #00F0FF; font-size: 28px; font-weight: bold; border: none; background: transparent;")
            val.setAlignment(Qt.AlignCenter)
            
            flayout.addWidget(title)
            flayout.addWidget(val)
            
            self.labels[key] = val
            grid.addWidget(frame, i // 4, i % 4) # 4 列排布
            
        layout.addLayout(grid)

    def update_data(self, data_dict):
        for k, v in data_dict.items():
            if k in self.labels:
                self.labels[k].setText(str(v))

class MapPage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(create_back_button(main_window))

        self.map_label = QLabel("[ WAITING FOR RADAR SLAM SCAN... ]")
        self.map_label.setStyleSheet("background-color: #05080F; color: #00F0FF; border: 2px dashed rgba(0, 240, 255, 0.5); font-size: 20px; letter-spacing: 2px;")
        self.map_label.setAlignment(Qt.AlignCenter)
        self.map_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.map_label)

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
        if not self.base_map_pixmap: return
        display_pixmap = self.base_map_pixmap.copy()

        if self.robot_pose and self.map_info:
            ox, oy, res = self.map_info
            rx, ry = self.robot_pose
            px = int((rx - ox) / res)
            py = display_pixmap.height() - int((ry - oy) / res)
            
            painter = QPainter(display_pixmap)
            painter.setBrush(QBrush(QColor(255, 0, 255))) 
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(px - 3, py - 3, 6, 6) 
            painter.setPen(QPen(QColor(255, 0, 255), 1))
            painter.drawEllipse(px - 8, py - 8, 16, 16)
            painter.end()

        target_w, target_h = self.map_label.width() - 4, self.map_label.height() - 4
        if target_w > 0 and target_h > 0:
            self.map_label.setPixmap(display_pixmap.scaled(target_w, target_h, Qt.KeepAspectRatio, Qt.SmoothTransformation))

class VoicePage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        layout = QVBoxLayout(self)
        layout.addWidget(create_back_button(main_window))

        lbl = QLabel("❖ 云端 AI 双脑神经枢纽 ❖")
        lbl.setStyleSheet("color: #FF00FF; font-size: 22px; font-weight: bold; margin-bottom: 5px;")
        lbl.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl)

        self.log = QTextBrowser()
        self.log.setText("> 神经交互网络已待命...\n")
        self.log.setStyleSheet("""
            QTextBrowser {
                color: #00F0FF; 
                background: rgba(5, 8, 15, 0.8); 
                border: 2px solid rgba(0, 240, 255, 0.4); 
                border-radius: 8px;
                padding: 10px; 
                font-size: 16px;
                line-height: 1.5;
            }
        """)
        self.log.document().setMaximumBlockCount(50) 
        layout.addWidget(self.log, stretch=1)

        # 麦克风状态按钮（初始为开启状态）
        self.mic_on_style = """
            QPushButton { 
                background-color: rgba(0, 255, 128, 0.08); 
                border: 2px solid #00FF80; 
                border-radius: 8px;
                color: #00FF80; 
                padding: 15px; 
                font-size: 20px; 
                font-weight: bold;
                letter-spacing: 2px;
            } 
            QPushButton:hover { background-color: #00FF80; color: #000; }
        """
        self.mic_off_style = """
            QPushButton { 
                background-color: rgba(255, 0, 60, 0.08); 
                border: 2px solid #FF003C; 
                border-radius: 8px;
                color: #FF003C; 
                padding: 15px; 
                font-size: 20px; 
                font-weight: bold;
                letter-spacing: 2px;
            } 
            QPushButton:hover { background-color: #FF003C; color: #000; }
        """
        self.mic_btn = QPushButton("🟢 听觉雷达运行中 (点击关闭)")
        self.mic_btn.setStyleSheet(self.mic_on_style)
        self.mic_btn.clicked.connect(self.main_window.ros_thread.send_voice_trigger)
        layout.addWidget(self.mic_btn)

    def update_mic_button(self, state_on):
        """根据麦克风状态更新按钮文字和样式"""
        if state_on:
            self.mic_btn.setText("🟢 听觉雷达运行中 (点击关闭)")
            self.mic_btn.setStyleSheet(self.mic_on_style)
        else:
            self.mic_btn.setText("🔴 听觉雷达已切断 (点击开启)")
            self.mic_btn.setStyleSheet(self.mic_off_style)

    def append_log(self, text):
        # 解析状态消息，更新按钮
        if text == "STATUS:ON":
            self.update_mic_button(True)
            return
        if text == "STATUS:OFF":
            self.update_mic_button(False)
            return
        if text == "CLEAR_SCREEN":
            self.log.clear()
            self.log.append("> 神经交互网络已重置，小爱同学正在聆听...\n")
            return
        self.log.append(f"> {text}")
        scrollbar = self.log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

# ==========================================
# 模块 C：系统主舞台
# ==========================================
class OS_MainStage(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("K1-AMR TERMINAL OS v3.0 (横屏适配版)")
        self.resize(1024, 600)
        self.setStyleSheet("background-color: #0A0E17; font-family: Consolas, 'Microsoft YaHei';")

        self.ros_thread = Ros2EngineThread()
        self.ros_thread.start()

        self.stacked_widget = QStackedWidget()
        self.setCentralWidget(self.stacked_widget)

        self.create_main_menu()
        
        self.cpu_page = CpuPage(self)
        self.nav_page = NavPage(self)
        self.esp32_page = Esp32Page(self)
        self.air_page = AirDataPage(self)
        self.map_page = MapPage(self)
        self.voice_page = VoicePage(self)

        self.ros_thread.map_signal.connect(self.map_page.update_map_data)
        self.ros_thread.pose_signal.connect(self.map_page.update_robot_pose)
        self.ros_thread.air_data_signal.connect(self.air_page.update_data)
        self.ros_thread.voice_log_signal.connect(self.voice_page.append_log)

        self.stacked_widget.addWidget(self.main_menu_widget) # 0
        self.stacked_widget.addWidget(self.cpu_page)         # 1
        self.stacked_widget.addWidget(self.nav_page)         # 2
        self.stacked_widget.addWidget(self.esp32_page)       # 3
        self.stacked_widget.addWidget(self.air_page)         # 4
        self.stacked_widget.addWidget(self.map_page)         # 5
        self.stacked_widget.addWidget(self.voice_page)       # 6

    def create_main_menu(self):
        self.main_menu_widget = QWidget()
        layout = QGridLayout(self.main_menu_widget)
        # 💡 横屏终极适配：缩小一点缝隙，防止按钮越界
        layout.setSpacing(15)
        layout.setContentsMargins(30, 20, 30, 30)

        title_lbl = QLabel("❖ K1-AMR 战术主控矩阵 ❖")
        title_lbl.setAlignment(Qt.AlignCenter)
        title_lbl.setStyleSheet("color: #00F0FF; font-size: 32px; font-weight: bold; letter-spacing: 5px; margin-bottom: 10px;")
        # 💡 跨越 3 列，镇住全局！
        layout.addWidget(title_lbl, 0, 0, 1, 3) 

        buttons_info = [
            ("📊\nCPU 算力引擎", 1),
            ("🎯\n战术导航下发", 2),
            ("💡\nESP32 物联阵列", 3),
            ("🌫️\n七合一环境雷达", 4),
            ("🗺️\nSLAM 实时测绘", 5),
            ("🎤\nAI 语音指挥中枢", 6)
        ]

        btn_style = """
            QPushButton { 
                background-color: rgba(0, 240, 255, 0.04); 
                border: 2px solid rgba(0, 240, 255, 0.5); 
                border-radius: 12px; 
                color: #00F0FF; 
                font-size: 20px; 
                font-weight: bold; 
                letter-spacing: 2px;
            }
            QPushButton:hover { 
                background-color: rgba(0, 240, 255, 0.9); 
                color: #000000; 
                border: 2px solid #FFFFFF; 
            }
        """

        # 💡 横屏终极适配：改成 3 列 2 行排布！彻底解决第三行被吞掉的问题！
        for i, (text, page_idx) in enumerate(buttons_info):
            btn = QPushButton(text)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            btn.setStyleSheet(btn_style)
            btn.clicked.connect(lambda checked, idx=page_idx: self.switch_to_page(idx))
            # 放入第 (i // 3) + 1 行，第 i % 3 列
            layout.addWidget(btn, (i // 3) + 1, i % 3)

    def switch_to_page(self, index):
        self.stacked_widget.setCurrentIndex(index)

    def closeEvent(self, event):
        self.ros_thread.stop()
        self.ros_thread.wait()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = OS_MainStage()
    window.showMaximized() 
    sys.exit(app.exec_())