import sys
import time
import psutil
import numpy as np

# --- ROS2 相关库 ---
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist 
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# --- PyQt5 相关库 ---
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QProgressBar, QPushButton, QGroupBox, QSizePolicy, QStackedWidget, QGridLayout)
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QImage, QPixmap, QPainter, QBrush, QColor, QPen
import pyqtgraph as pg

# ==========================================
# 模块 A：后台 ROS2 综合通信引擎
# ==========================================
class Ros2EngineThread(QThread):
    map_signal = pyqtSignal(object, float, float, float)
    pose_signal = pyqtSignal(float, float) 
    
    # 新增占位信号 (空气数据、语音状态等)
    air_data_signal = pyqtSignal(dict)
    voice_log_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('gui_comprehensive_engine')
        
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.map_sub = self.node.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.pose_sub = self.node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.goal_pub = self.node.create_publisher(PoseStamped, '/goal_pose', 10)
        
        self.current_x = 0.0
        self.current_y = 0.0

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
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.pose_signal.emit(self.current_x, self.current_y)

    def send_goal(self, target_x, target_y):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = float(target_x)
        goal_msg.pose.position.y = float(target_y)
        goal_msg.pose.orientation.w = 1.0 
        self.goal_pub.publish(goal_msg)
        print(f"🚀 [控制塔] 已下发导航目标: X={target_x:.2f}, Y={target_y:.2f}", flush=True)

    def send_esp32_cmd(self, cmd):
        # 这里预留给 MQTT 或 ROS 发布，控制 ESP32
        print(f"💡 [物联网] 发送 ESP32 灯光指令: {cmd}", flush=True)

    def run(self):
        # 模拟产生一些空气传感器数据供 UI 显示
        self.sim_timer = self.node.create_timer(2.0, self.simulate_air_data)
        rclpy.spin(self.node)

    def simulate_air_data(self):
        # 模拟 7 合 1 传感器数据
        data = {"Temp": 25.1, "Hum": 60.5, "PM25": 35, "PM10": 42, "CO2": 800, "VOC": 120, "HCHO": 8}
        self.air_data_signal.emit(data)

    def stop(self):
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.quit()

# ==========================================
# 模块 B：前端子页面组件
# ==========================================

def create_back_button(parent):
    """生成统一的返回按钮"""
    btn = QPushButton("< 返回主控台 (RETURN)")
    btn.setStyleSheet("""
        QPushButton { background-color: #0A0E17; border: 1px solid #FF003C; color: #FF003C; padding: 10px; font-weight: bold; font-family: Consolas;}
        QPushButton:hover { background-color: #FF003C; color: #000000; }
    """)
    btn.clicked.connect(lambda: parent.switch_to_page(0))
    return btn

# 页面 1：CPU 监控
class CpuPage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(create_back_button(main_window))
        
        self.plot_widget = pg.PlotWidget(title="< SYSTEM.CPU_LOAD >")
        self.plot_widget.setYRange(0, 100)
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.getAxis('left').setPen('#00F0FF')
        self.plot_widget.getAxis('bottom').setPen('#00F0FF')
        self.plot_widget.setBackground('#0A0E17') 
        layout.addWidget(self.plot_widget) 

        self.num_points = 50 
        self.cpu_data = np.zeros((8, self.num_points))
        self.curves = []
        neon_colors = [(0, 240, 255), (255, 0, 255), (0, 255, 128), (255, 255, 0), (255, 80, 80), (180, 0, 255), (255, 165, 0), (200, 200, 255)]
        
        for i in range(8):
            curve = self.plot_widget.plot(self.cpu_data[i], pen=pg.mkPen(color=neon_colors[i], width=1.5))
            self.curves.append(curve)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_cpu_data)
        self.timer.start(500)

    def update_cpu_data(self):
        usages = psutil.cpu_percent(interval=None, percpu=True)
        for i in range(8):
            self.cpu_data[i] = np.roll(self.cpu_data[i], -1)
            self.cpu_data[i][-1] = usages[i]
            self.curves[i].setData(self.cpu_data[i])

# 页面 2：战术导航
class NavPage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        layout = QVBoxLayout(self)
        layout.addWidget(create_back_button(main_window))
        
        lbl = QLabel(":: NAV_OVERRIDE ::\n选择目标点下发指令")
        lbl.setStyleSheet("color: #00F0FF; font-size: 20px; font-weight: bold;")
        lbl.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl)

        btn_style = "QPushButton { background-color: transparent; border: 2px solid #00F0FF; color: #00F0FF; padding: 20px; font-size: 18px; } QPushButton:hover { background-color: #00F0FF; color: #000; }"
        
        btn_a = QPushButton("[ A 点: 充电站 ]")
        btn_a.setStyleSheet(btn_style)
        btn_a.clicked.connect(lambda: self.main_window.ros_thread.send_goal(1.0, 1.0))
        layout.addWidget(btn_a)

        btn_b = QPushButton("[ B 点: 巡逻区 ]")
        btn_b.setStyleSheet(btn_style)
        btn_b.clicked.connect(lambda: self.main_window.ros_thread.send_goal(2.0, 2.0))
        layout.addWidget(btn_b)

# 页面 3：ESP32 物联网控制
class Esp32Page(QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        layout = QVBoxLayout(self)
        layout.addWidget(create_back_button(main_window))
        
        lbl = QLabel(":: ESP32 IOT CONTROL ::")
        lbl.setStyleSheet("color: #00F0FF; font-size: 24px; font-weight: bold;")
        lbl.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl)

        btn_on = QPushButton("💡 强行点亮 (ON)")
        btn_on.setStyleSheet("QPushButton { border: 2px solid #00FF80; color: #00FF80; padding: 30px; font-size: 24px; } QPushButton:hover { background-color: #00FF80; color: #000; }")
        btn_on.clicked.connect(lambda: self.main_window.ros_thread.send_esp32_cmd("ON"))
        
        btn_off = QPushButton("🌑 强行熄灭 (OFF)")
        btn_off.setStyleSheet("QPushButton { border: 2px solid #FF003C; color: #FF003C; padding: 30px; font-size: 24px; } QPushButton:hover { background-color: #FF003C; color: #000; }")
        btn_off.clicked.connect(lambda: self.main_window.ros_thread.send_esp32_cmd("OFF"))

        layout.addWidget(btn_on)
        layout.addWidget(btn_off)

# 页面 4：七合一雷达数据
class AirDataPage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(create_back_button(main_window))

        self.labels = {}
        grid = QGridLayout()
        keys = ["Temp", "Hum", "PM25", "PM10", "CO2", "VOC", "HCHO"]
        names = ["温度 (℃)", "湿度 (%)", "PM2.5", "PM10", "CO2 (ppm)", "VOC", "甲醛"]
        
        for i, key in enumerate(keys):
            title = QLabel(names[i])
            title.setStyleSheet("color: #888; font-size: 16px;")
            val = QLabel("--")
            val.setStyleSheet("color: #00F0FF; font-size: 32px; font-weight: bold;")
            self.labels[key] = val
            grid.addWidget(title, i//2, (i%2)*2)
            grid.addWidget(val, i//2, (i%2)*2 + 1)
            
        layout.addLayout(grid)

    def update_data(self, data_dict):
        for k, v in data_dict.items():
            if k in self.labels:
                self.labels[k].setText(str(v))

# 页面 5：地图与定位
class MapPage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(create_back_button(main_window))

        self.map_label = QLabel("[ ACQUIRING RADAR SCAN... ]")
        self.map_label.setStyleSheet("background-color: #05080F; color: #00F0FF; border: 1px dashed #00F0FF;")
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
            painter.drawEllipse(px - 6, py - 6, 12, 12)
            painter.end()

        target_w, target_h = self.map_label.width() - 4, self.map_label.height() - 4
        if target_w > 0 and target_h > 0:
            self.map_label.setPixmap(display_pixmap.scaled(target_w, target_h, Qt.KeepAspectRatio, Qt.SmoothTransformation))

# 页面 6：语音与 AI 通信
class VoicePage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(create_back_button(main_window))

        self.log = QLabel("> 等待语音唤醒...\n")
        self.log.setStyleSheet("color: #00F0FF; background: #05080F; border: 1px solid #222; padding: 10px;")
        self.log.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        layout.addWidget(self.log, stretch=1)

        btn = QPushButton("🎤 [ 按住启动神经交互 ]")
        btn.setStyleSheet("QPushButton { border: 2px solid #FF00FF; color: #FF00FF; padding: 30px; font-size: 20px; font-weight: bold;} QPushButton:hover { background-color: #FF00FF; color: #000; }")
        layout.addWidget(btn)

# ==========================================
# 模块 C：系统主舞台 (OS 容器)
# ==========================================
class OS_MainStage(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("K1-AMR TERMINAL OS v2.0")
        self.resize(1024, 600)
        self.setStyleSheet("background-color: #0A0E17; font-family: Consolas;")

        # 启动后台引擎
        self.ros_thread = Ros2EngineThread()
        self.ros_thread.start()

        # 核心：栈式窗口管理器
        self.stacked_widget = QStackedWidget()
        self.setCentralWidget(self.stacked_widget)

        # 实例化所有页面
        self.create_main_menu()
        
        self.cpu_page = CpuPage(self)
        self.nav_page = NavPage(self)
        self.esp32_page = Esp32Page(self)
        self.air_page = AirDataPage(self)
        self.map_page = MapPage(self)
        self.voice_page = VoicePage(self)

        # 绑定后台信号到各个页面
        self.ros_thread.map_signal.connect(self.map_page.update_map_data)
        self.ros_thread.pose_signal.connect(self.map_page.update_robot_pose)
        self.ros_thread.air_data_signal.connect(self.air_page.update_data)

        # 把页面装入栈中 (索引 0~6)
        self.stacked_widget.addWidget(self.main_menu_widget) # 0
        self.stacked_widget.addWidget(self.cpu_page)         # 1
        self.stacked_widget.addWidget(self.nav_page)         # 2
        self.stacked_widget.addWidget(self.esp32_page)       # 3
        self.stacked_widget.addWidget(self.air_page)         # 4
        self.stacked_widget.addWidget(self.map_page)         # 5
        self.stacked_widget.addWidget(self.voice_page)       # 6

    def create_main_menu(self):
        """创建酷炫的六宫格主菜单"""
        self.main_menu_widget = QWidget()
        layout = QGridLayout(self.main_menu_widget)
        layout.setSpacing(15)
        layout.setContentsMargins(20, 20, 20, 20)

        buttons_info = [
            ("📊\nCPU 性能矩阵", 1),
            ("🎯\n战术导航下发", 2),
            ("💡\nESP32 物联阵列", 3),
            ("🌫️\n七合一环境雷达", 4),
            ("🗺️\nSLAM 实时测绘", 5),
            ("🎤\nAI 语音指挥中枢", 6)
        ]

        btn_style = """
            QPushButton { background-color: rgba(0, 240, 255, 0.05); border: 2px solid #00F0FF; border-radius: 8px; color: #00F0FF; font-size: 22px; font-weight: bold; }
            QPushButton:hover { background-color: #00F0FF; color: #000000; border: 2px solid #FFFFFF; }
        """

        for i, (text, page_idx) in enumerate(buttons_info):
            btn = QPushButton(text)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            btn.setStyleSheet(btn_style)
            # Python 闭包陷阱：强制绑定当前循环的 idx
            btn.clicked.connect(lambda checked, idx=page_idx: self.switch_to_page(idx))
            layout.addWidget(btn, i // 2, i % 2)

    def switch_to_page(self, index):
        """核心路由引擎：切换页面"""
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