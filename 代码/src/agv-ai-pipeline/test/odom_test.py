# odom_subscriber.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf_transformations
import threading
from rclpy.executors import MultiThreadedExecutor
import math 

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        # 订阅里程计（用于获取当前朝向）
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.current_yaw = 0.0  # 当前朝向
        self.lock = threading.Lock()  # 锁，用于线程逻辑同步

    def odom_callback(self, msg):
        """解析里程计数据，更新当前机器人朝向"""
        _q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([_q.x, _q.y, _q.z, _q.w])
        print(f"里程计角度: {math.degrees(self.current_yaw) + 180}")
        with self.lock:
            self.current_yaw = yaw



def main():
    rclpy.init()

    # 创建 OdomSubscriber 节点
    node = OdomSubscriber()

    # 使用 MultiThreadedExecutor 运行 ROS 事件循环
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()  # 启动 ROS 事件循环
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
