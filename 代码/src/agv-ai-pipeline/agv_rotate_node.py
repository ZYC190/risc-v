import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

import tf_transformations
import threading
import time
import math
from rclpy.executors import MultiThreadedExecutor
from agv_common_class import AngleSubscriber, OdomSubscriber, RotateRobot

def run_executor(executor):
    executor.spin()  # 运行 ROS 2 事件循环（不会阻塞主线程）

def main():
    rclpy.init()

    angle_sub_node = AngleSubscriber()  # 声源定位信息订阅
    odom_sub_node = OdomSubscriber()    # 里程计信息订阅
    node_rotate = RotateRobot(odom_sub_node, angle_sub_node)  # 传递订阅节点的引用

    # 多线程执行的管理器
    executor = MultiThreadedExecutor()
    executor.add_node(angle_sub_node)
    executor.add_node(odom_sub_node)
    executor.add_node(node_rotate)

    executor_thread = threading.Thread(target=run_executor, args=(executor,), daemon=True)
    executor_thread.start()

    try:
        while rclpy.ok():
            print("等待唤醒...")
            angle_sub_node.ready_to_wait_event.set()
            angle_sub_node.trigger_event.wait()
            angle_sub_node.trigger_event.clear()

            node_rotate.rotate_to_angle()  # 触发旋转
            print("转向完毕")
    except KeyboardInterrupt:
        print("程序终止")

    finally:
        node_rotate.stop_move()
        time.sleep(0.1)
        executor.shutdown()
        if rclpy.ok():  # 只有在 rclpy 仍然运行时才调用 shutdown
            rclpy.shutdown()

if __name__ == '__main__':
    main()
