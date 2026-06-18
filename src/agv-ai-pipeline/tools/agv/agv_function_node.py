import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
import tf_transformations
import threading
import time
import math
import difflib


class FuncControllerNode(Node):
    def __init__(self):
        super().__init__('func_controller_node')

        # ros2 相关
        # 跟踪服务
        self.client = self.create_client(SetBool, 'toggle_follow')
        self.service_available = False
        for i in range(3):
            if self.client.wait_for_service(timeout_sec=1.0):
                self.service_available = True
                self.get_logger().info('toggle_follow 服务已连接')
                break
            self.get_logger().info(f'等待服务可用中... ({i+1}/3)')
        if not self.service_available:
            self.get_logger().warn('toggle_follow 服务不可用，跟随功能将不可使用')
        self.request = SetBool.Request()

        # 速度发布者
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 30)

        self.lock = threading.Lock() 
    
        # function calling
        self.function_dict = {
            "go_to_destination": self.go_to_destination,
            "go_to_oringin": self.go_to_oringin,
            "move_away" : self.move_away,

            "come_here":self.empty_func,
            "follow_me": self.follow_me,
            "come_on": self.follow_me,
            "go_away": self.stop_follow_me,
            "stop_follow_me": self.stop_follow_me,
            "take_two_steps_forward": self.take_two_steps_forward,
            "take_two_steps_backward": self.take_two_steps_backward,
            "rotate_in_place": self.rotate_in_place,
            "go_forward": self.go_forward,
            "go_backward": self.go_backward,
            "turn_left": self.turn_left,
            "turn_left_circle": self.turn_left_circle,
            "turn_right": self.turn_right,
            "turn_right_circle": self.turn_right_circle,
            "stand_up": self.stand_up,
            "lie_down": self.lie_down,
            "roll_over": self.roll_over,
            "wiggle_hips": self.wiggle_hips,
            "turn_head_left": self.turn_head_left,
            "turn_head_right": self.turn_head_right,
            "wag_tail": self.wag_tail,
            "moonwalk": self.moonwalk,
            "army_crawl": self.army_crawl,
            "do_flip": self.do_flip,
            "show_dance": self.show_dance
        }

        self.function_dict_zh = {
            "小微，把快递送给马哥": "go_to_destination",
            "送给马哥": "go_to_destination",

            "赏你": "go_to_oringin",
            "回去吧": "go_to_oringin",
            "收到了": "go_to_oringin",

            "别在这呆着了": 'move_away',
            "去其它地方吧": 'move_away',

            "过来":"come_here",
            "滚开":"stop_follow_me",

            "跟我走":"follow_me",
            "小微，跟我走":"follow_me",
            "小微小微，跟我走":"follow_me",
            "跟着我":"follow_me",
            
            "停止跟随":"stop_follow_me",
            "不要跟着我":"stop_follow_me",
            
            "向前走两步":"take_two_steps_forward",
            "离我近一点":"take_two_steps_forward",
            "向后走两步":"take_two_steps_backward",
            "离我远一点":"take_two_steps_backward",

            "后退":"take_two_steps_backward",
            "往后走":"take_two_steps_backward",

            "原地旋转":"rotate_in_place",
            "小微，来个转圈表演": "rotate_in_place",
            "小微，转个圈": "rotate_in_place",
            "转个圈": "rotate_in_place",
            "来个转圈表演": "rotate_in_place",

            "向前走":"go_forward",
            "向后走":"go_backward",
            "向左转":"turn_left",
            "向右转":"turn_right",
            
            "原地左转圈":"turn_left_circle",
            "原地右转圈":"turn_right_circle"
        }

    ## 对接到导航堆栈
    def go_to_destination(self):
        pass

    def go_to_oringin(self):
        pass

    def move_away(self):
        self.rotate_in_place()
        self.take_two_steps_backward()
        pass

    def empty_func(self):
        pass

    ## ros2 相关函数
    # 跟踪请求发送
    def send_track_request(self, enable):
        if not self.service_available:
            self.get_logger().warn(f'跟踪服务不可用，跳过请求: {"启用" if enable else "停用"}')
            return
        self.request.data = enable
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        ret = future.result()
        self.get_logger().info(f'服务响应: success={ret.success}, message="{ret.message}"')

    # 停止机器人运动
    def robot_stop_move(self):
        self.send_track_request(False)
        rate = 30
        twist = Twist()
        for i in range(3):
            self.cmd_vel_publisher.publish(twist)
            time.sleep(1.0 / float(rate))
        print("STOP MOVE")

    # 旋转以0.2角速度
    def robot_rotate_02(self, angle=360, direction=1):
        time_360_rotate = 14.6 # 旋转的时间
        rotate_time = time_360_rotate * float(angle) / 360.0
        twist = Twist()
        twist.angular.z = 0.2 * direction
        self.cmd_vel_publisher.publish(twist)
        time.sleep(rotate_time)

    # 旋转以1.0角速度
    def robot_rotate_010(self, angle=360, direction=1):
        time_360_rotate = 4.3 # 旋转的时间
        rotate_time = time_360_rotate * float(angle) / 360.0
        twist = Twist()
        twist.angular.z = 1.5 * direction
        self.cmd_vel_publisher.publish(twist)
        time.sleep(rotate_time)

    def robot_xy_move_02(self, x_speed = 0.2, y_speed = 0.2, direction_x = 1, direction_y = 1, time_cost = 2):
        twist = Twist()
        twist.linear.x = x_speed * float(direction_x)
        twist.linear.y = y_speed * float(direction_y)
        self.cmd_vel_publisher.publish(twist)
        time.sleep(time_cost)

    # 函数调用
    def follow_me(self):
        """跟随"""
        self.send_track_request(True)
        print("Following the user.")

    def stop_follow_me(self):
        """停止跟随"""
        self.send_track_request(False)
        print("Stopped following the user.")

    def take_two_steps_forward(self):
        """向前走两步"""
        self.robot_stop_move()
        self.robot_xy_move_02(x_speed = 0.2, y_speed = 0.0, direction_x = 1, direction_y = 1, time_cost = 2)
        self.robot_stop_move()
        print("Taking two steps forward.")

    def take_two_steps_backward(self):
        """向后走两步"""
        self.robot_stop_move()
        self.robot_xy_move_02(x_speed = 0.3, y_speed = 0.0, direction_x = -1, direction_y = 1, time_cost = 2)
        self.robot_stop_move()
        print("Taking two steps backward.")

    def rotate_in_place(self):
        """原地旋转"""
        print("Rotating in place.")
        self.robot_stop_move()
        self.robot_rotate_010(angle=428, direction=1)
        self.robot_stop_move()
        
    def go_forward(self):
        """向前走"""
        self.robot_stop_move()
        self.robot_xy_move_02(x_speed = 0.1, y_speed = 0.0, direction_x = 1, direction_y = 1, time_cost = 0)
        print("Moving forward.")

    def go_backward(self):
        """向后走"""
        self.robot_stop_move()
        self.robot_xy_move_02(x_speed = 0.1, y_speed = 0.0, direction_x = -1, direction_y = 1, time_cost = 0)
        print("Moving backward.")

    def turn_left(self):
        """向左转"""
        self.robot_stop_move()
        self.robot_rotate_010(angle=180, direction=-1)
        self.robot_stop_move()
        print("Turning left.")

    def turn_right(self):
        """向右转"""
        self.robot_stop_move()
        self.robot_rotate_010(angle=180, direction=1)
        self.robot_stop_move()
        print("Turning right.")

    def turn_left_circle(self):
        print("Turning left in a circle.")
        self.robot_stop_move()
        self.robot_rotate_010(angle=360, direction=1)
        self.robot_stop_move()

    def turn_right_circle(self):
        print("Turning right in a circle.")
        self.robot_stop_move()
        self.robot_rotate_010(angle=360, direction=-1)
        self.robot_stop_move()

    def stand_up(self):
        print("Standing up.")

    def lie_down(self):
        print("Lying down forward.")

    def roll_over(self):
        print("Rolling over backward.")

    def wiggle_hips(self):
        print("Wiggling hips.")

    def turn_head_left(self):
        print("Turning head left.")

    def turn_head_right(self):
        print("Turning head right.")

    def wag_tail(self):
        print("Wagging tail.")

    def moonwalk(self):
        print("Performing moonwalk.")

    def army_crawl(self):
        print("Crawling forward like a soldier.")

    def do_flip(self):
        print("Doing a flip.")

    def show_dance(self):
        print("Showing a dance move.")


