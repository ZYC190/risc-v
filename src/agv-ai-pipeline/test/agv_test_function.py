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

# ours
from tools.agv import FuncControllerNode, execute_command


def run_executor(executor):
    executor.spin()  # 运行 ROS 2 事件循环（不会阻塞主线程）

def main():
    rclpy.init()

    func_controller_node = FuncControllerNode() # 总控制

    # 多线程执行的管理器
    executor = MultiThreadedExecutor()
    executor.add_node(func_controller_node)

    executor_thread = threading.Thread(target=run_executor, args=(executor,), daemon=True)
    executor_thread.start()
    try:
        while rclpy.ok():
            user_input = input("请输入函数名: ")
            ret, func_name = execute_command(user_input, func_controller_node.function_dict_zh)
            if func_name in func_controller_node.function_dict:
                func_controller_node.function_dict[func_name]()
            else:
                print("输入的函数名错误")

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("程序终止")

    finally:
        time.sleep(0.1)
        executor.shutdown()
        if rclpy.ok():  # 只有在 rclpy 仍然运行时才调用 shutdown
            rclpy.shutdown()

if __name__ == '__main__':
    main()
