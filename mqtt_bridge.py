import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import paho.mqtt.client as mqtt
import json
import math

class MqttRosBridge(Node):
    def __init__(self):
        super().__init__('mqtt_ros_bridge')
        
        # 1. 保留底层遥控的对讲机
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 2. 新增：连接 Nav2 导航大模型的“派单专线” (Action Client)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 3. 启动 MQTT 监听
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        
        self.get_logger().info("⏳ 正在连接 MQTT 基站...")
        self.mqtt_client.connect("localhost", 1883, 60)
        self.mqtt_client.loop_start()
        
    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("✅ 成功连上基站！正在监听 robot/action 频道...")
        client.subscribe("robot/action")
        
    def on_message(self, client, userdata, msg):
        payload = msg.payload.decode('utf-8')
        self.get_logger().info(f"📡 收到云端指令: {payload}")
        try:
            data = json.loads(payload)
            action = data.get("action", "")
            
            # 如果是导航派单指令
            if action == "nav_to":
                x = data.get("x", 0.0)
                y = data.get("y", 0.0)
                yaw = data.get("yaw", 0.0) # 车头朝向角度(弧度)
                self.send_nav_goal(x, y, yaw)
            # 否则当做普通遥控指令
            else:
                self.send_cmd_vel(action)
                
        except Exception as e:
            self.get_logger().error(f"❌ 指令解析失败: {e}")

    def send_nav_goal(self, x, y, yaw):
        self.get_logger().info("🔍 正在呼叫 Nav2 导航服务...")
        # 等待导航系统准备就绪
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("❌ Nav2 导航服务未启动，请检查 launch 文件！")
            return
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # 填入云端下发的坐标
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        
        # 把普通角度转换为四元数(机器人的方向逻辑)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.get_logger().info(f"🚀 派单成功！目标坐标 -> X: {x}, Y: {y}")
        self.nav_to_pose_client.send_goal_async(goal_msg)

    def send_cmd_vel(self, action):
        twist = Twist()
        speed = 0.2
        if action == 'forward': twist.linear.x = speed
        elif action == 'backward': twist.linear.x = -speed
        elif action == 'left': twist.linear.y = speed
        elif action == 'right': twist.linear.y = -speed
        elif action == 'turn_left': twist.angular.z = speed * 2
        elif action == 'turn_right': twist.angular.z = -speed * 2
        elif action == 'stop': pass
        else: return
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MqttRosBridge()
    rclpy.spin(node)
    node.mqtt_client.loop_stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()