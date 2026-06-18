import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from tf_transformations import quaternion_from_euler
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

import tf_transformations
import threading
import time
import math

from geometry_msgs.msg import PoseStamped, Quaternion
import tf2_ros






class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def create_pose(self, x: float, y: float, yaw: float = 0.0) -> PoseStamped:
        """生成导航目标点 PoseStamped（地图坐标系下）"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def navigate_to(self, pose: PoseStamped) -> bool:
        self._client.wait_for_server()
        goal = NavigateToPose.Goal()
        goal.pose = pose

        send_goal_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("❌ 目标未被接受")
            return False

        self.get_logger().info("✅ 目标已被接受，导航中...")
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result

        self.get_logger().info(f"🎯 导航完成，状态码：{result.result}")
        return True

class AngleSubscriber(Node):
    def __init__(self):
        super().__init__('angle_subscriber')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'angle_topic',
            self.listener_callback,
            10
        )
        self.angle = 0
        self.lock = threading.Lock() # 锁，用于线程数据同步
        self.trigger_event = threading.Event()

        # 外部会设置这个，在主线程 wait 前调用 ready_to_wait_event.set()
        self.ready_to_wait_event = threading.Event()

    def listener_callback(self, msg):
        with self.lock:
            if msg.data and len(msg.data) >= 2:
                self.angle = msg.data[0]
                if msg.data[1] == 1:
                    # print("Trigger received! Angle:", self.angle)
                    # self.trigger_event.set()

                    # 只有主线程已经 set() 表示“准备等待”，我们才触发
                    if self.ready_to_wait_event.is_set():
                        print("Trigger received! Angle:", self.angle)
                        self.trigger_event.set()

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
        self.lock = threading.Lock() 

    def odom_callback(self, msg):
        """解析里程计数据，更新当前机器人朝向"""
        _q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([_q.x, _q.y, _q.z, _q.w])
        # print(f"里程计角度: {math.degrees(self.current_yaw) + 180}")
        with self.lock:
            self.current_yaw = yaw

class RotateRobot(Node):
    def __init__(self, odom_subscriber, angle_subscriber):
        super().__init__('rotate_robot')

        self.odom_subscriber = odom_subscriber  # 里程计订阅节点
        self.angle_subscriber = angle_subscriber  # 角度订阅节点
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 20)

    def rotate_to_angle(self):
        """让机器人旋转到目标角度"""
        with self.angle_subscriber.lock:
            audio_angle = self.angle_subscriber.angle  # 获取目标角度

        if audio_angle is None or audio_angle == 0 or audio_angle == 360:
            return

        with self.odom_subscriber.lock:
            odom_start_raw = math.degrees(self.odom_subscriber.current_yaw) + 180  # 获取当前朝向, 转换到 0 - 360 度

         # 计算旋转方向和目标角度
        if 0 < audio_angle < 180:  # 顺时针旋转
            direction = -1.0
            rotate_angle = audio_angle
        else:  # 逆时针旋转
            direction = 1.0
            rotate_angle = 360 - audio_angle

        # 里程计目标角度计算
        if direction < 0:
            odom_target_raw = odom_start_raw - rotate_angle
            if odom_target_raw < 0:
                odom_target_raw += 360
        else:
            odom_target_raw = odom_start_raw + rotate_angle
            if odom_target_raw > 360:
                odom_target_raw -= 360

        print(f"要旋转: {rotate_angle}°，方向: {direction}，当前角度: {odom_start_raw}°，目标角度: {odom_target_raw}°")

        full_error = odom_start_raw - odom_target_raw
        if full_error < 0:
            signed_flag = -1.0
        else:
            signed_flag = 1.0

        
        # circle_time = 14.5 # 一圈的时间
        # rotate_time = circle_time * rotate_angle / 360.0
        # print(f"旋转时间: {rotate_time}")

        speed_base = 0.6
        twist = Twist()
        twist.angular.z = speed_base * direction
        self.cmd_vel_publisher.publish(twist)

        # time.sleep(rotate_time)

        while True:
            with self.odom_subscriber.lock:
                current_yaw = math.degrees(self.odom_subscriber.current_yaw) + 180  # 更新当前里程计角度

            error = current_yaw - odom_target_raw

            # print(error)
            if abs(error) < 6:
                self.cmd_vel_publisher.publish(Twist())
                break

            time.sleep(0.08)
        # 停止机器人
        for i in range(0, 3):
            self.cmd_vel_publisher.publish(Twist())
        self.get_logger().info("Rotation complete.")

    def stop_move(self):
        # 停止机器人
        self.cmd_vel_publisher.publish(Twist())


class AngleNavGoalPoseSend(Node):
    def __init__(self, angle_sub, invert=False, distance=2.5):
        super().__init__('goal_pose_send_node')
        self.get_logger().info(f"导航控制节点启动")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.angle_sub = angle_sub
        self.goal_pose = None

        self.goal_tuple = (0, 0, 0)
        self.invert = invert

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.distance = distance

    def deg360_to_rad_pi(self, angle_deg):
        """
        将 0~360 度（逆时针为正）转换为 -π~π 弧度
        """
        angle_deg = angle_deg % 360  # 保证角度在 0~360 之间
        if angle_deg > 180:
            angle_deg -= 360  # 转换为 [-180, 180]
        angle_rad = math.radians(angle_deg)
        return angle_rad

    def normalize_angle(self, angle):
        """将任意角度归一化到 [-pi, pi)"""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def calc_target_point(self):

        try:
            angle_deg = self.angle_sub # 0 - 360 逆时针
            if self.invert:
                angle_deg = 360 - angle_deg
            angle_rad = self.deg360_to_rad_pi(angle_deg)
            self.get_logger().info(f'Received angle: {angle_rad:.3f} rad')

            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', now, timeout=rclpy.duration.Duration(seconds=5))

            current_x = trans.transform.translation.x
            current_y = trans.transform.translation.y
            orientation_q = trans.transform.rotation

            _, _, yaw = tf_transformations.euler_from_quaternion([
                orientation_q.x,
                orientation_q.y,
                orientation_q.z,
                orientation_q.w
            ])

            print(f"current_x:{current_x}, current_y:{current_y}")

            angle_rad = self.normalize_angle(angle_rad + yaw)

            base_x = self.distance * math.cos(angle_rad)
            base_y = self.distance * math.sin(angle_rad)
            target_x = current_x + base_x
            target_y = current_y + base_y

            print(f"target_x:{target_x}, target_y:{target_y}")

            goal_pose = PoseStamped()
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = target_x
            goal_pose.pose.position.y = target_y
            goal_pose.pose.position.z = 0.0

            quat = self.euler_to_quaternion(0, 0, angle_rad)
            goal_pose.pose.orientation = quat

            self.goal_pose = goal_pose

            self.goal_tuple = (target_x, target_y, angle_rad)
        except Exception as e:
            self.get_logger().warn(f'Could not transform "base_footprint" to "map": {e}')

    def publish_pose(self):
        if self.goal_pose is None:
            print("导航计算失败")
            return

        target_x, target_y, angle_rad = self.goal_tuple
        self.get_logger().info(f'发布导航目标: x={target_x:.2f}, y={target_y:.2f}, yaw={angle_rad:.2f} rad')
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.goal_pose.pose.position.x
        pose.pose.position.y = self.goal_pose.pose.position.y
        pose.pose.orientation = self.goal_pose.pose.orientation

        self.goal_pub.publish(pose)


    def euler_to_quaternion(self, roll, pitch, yaw):
        q = Quaternion()

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q
