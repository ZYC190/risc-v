#!/usr/bin/env python3
"""
TF 坐标变换管理器 — 眼在手外 (Eye-to-Hand)

将双目相机发布的 PointStamped 从相机光学坐标系
变换到机械臂基座坐标系。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs


class TFManager:
    """
    管理从相机坐标系到机械臂基座坐标系的 TF 变换。

    使用 tf2_ros 的 Buffer + TransformListener 监听静态/动态变换。
    眼在手外 (Eye-to-Hand): 相机固定在外, 不随机械臂移动。
    """

    def __init__(self, node: Node, camera_frame: str, arm_base_frame: str):
        """
        Args:
            node: ROS 2 节点 (用于创建 tf2 组件)
            camera_frame: 相机光学坐标系名称 (如 "camera_color_optical_frame")
            arm_base_frame: 机械臂基座坐标系名称 (如 "table_arm_base_link")
        """
        self._node = node
        self.camera_frame = camera_frame
        self.arm_base_frame = arm_base_frame

        # tf2 缓冲区 + 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        self._node.get_logger().info(
            f'TFManager 已初始化: 监听变换 '
            f'"{camera_frame}" → "{arm_base_frame}"'
        )

    def transform_point(self, point_stamped: PointStamped) -> PointStamped:
        """
        将 PointStamped 从相机帧变换到机械臂基座帧。

        Args:
            point_stamped: 相机坐标系下的 3D 点
                           (header.frame_id 应为相机光学帧)

        Returns:
            机械臂基座坐标系下的 3D 点

        Raises:
            tf2_ros.TransformException: 如果变换不可用 (TF 未发布或超时)
        """
        # 确保输入点有正确的 frame_id
        if not point_stamped.header.frame_id:
            self._node.get_logger().warn(
                'PointStamped 缺少 header.frame_id, '
                f'假定为 "{self.camera_frame}"'
            )
            point_stamped.header.frame_id = self.camera_frame

        # 查找最新变换 (1秒超时)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.arm_base_frame,               # 目标帧
                point_stamped.header.frame_id,     # 源帧
                rclpy.time.Time(),                 # 最新可用
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except tf2_ros.LookupException as e:
            self._node.get_logger().error(
                f'TF 查找失败: 找不到从 '
                f'"{point_stamped.header.frame_id}" 到 "{self.arm_base_frame}" 的变换. '
                f'请确认 static_transform_publisher 已启动. 错误: {e}'
            )
            raise
        except tf2_ros.ConnectivityException as e:
            self._node.get_logger().error(
                f'TF 连接失败 (两个坐标系之间没有发布链): {e}'
            )
            raise
        except tf2_ros.ExtrapolationException as e:
            self._node.get_logger().error(
                f'TF 外推失败 (变换太旧): {e}'
            )
            raise

        # 执行变换
        return tf2_geometry_msgs.do_transform_point(point_stamped, transform)

    def can_transform(self, source_frame=None) -> bool:
        """
        检查变换是否可用 (不阻塞)。

        Args:
            source_frame: 源坐标系 (默认使用 camera_frame)

        Returns:
            True 如果变换可用
        """
        if source_frame is None:
            source_frame = self.camera_frame
        return self.tf_buffer.can_transform(
            self.arm_base_frame,
            source_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.1)
        )
