import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 1. 启动底层导航 (等同于 ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py)
    # 自动去找 wheeltec_nav2 这个包的安装路径
    nav2_launch_dir = get_package_share_directory('wheeltec_nav2')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'launch', 'wheeltec_nav2.launch.py'))
    )

    # 2. 启动语音节点 (等同于 ros2 run jobot_mic myagv_mic_node)
    mic_node = Node(
        package='jobot_mic',
        executable='myagv_mic_node',
        name='myagv_mic_node',
        output='screen'
    )

    # 3. 启动主控脚本 (等同于 python3 agv_master_node.py)
    # 👉 注意：这里一定要填你在 RISC-V 板子上的绝对路径！比如 /home/zyc/xxx/agv_master_node.py
    master_process = ExecuteProcess(
        cmd=['python3', '/home/zyc/agv-ai-pipeline/agv_master_node.py'],
        output='screen'
    )

    # 4. 启动 Qt 科幻控制面板 (等同于 python3 qt.py)
    # 👉 注意：同样填绝对路径！
# 4. 启动 Qt 科幻控制面板 (官方硬件加速版)
# 4. 启动 Qt 科幻控制面板 (物理斩断 SSH 专线版)
# 4. 启动 Qt 科幻控制面板 (官方硬件加速版)
    qt_process = ExecuteProcess(
        cmd=['python3', 'qt.py'],
        cwd='/home/zyc/agv-ai-pipeline/QT',
        additional_env={
            # 👉 绝杀招：强行清空 SSH 带来的虚拟显示器，彻底切断去电脑的退路！
            # 'DISPLAY': '', 
            
            # 👉 剩下的完全遵照官方：强制使用 Wayland 物理屏
            'QT_QPA_PLATFORM': 'wayland',    
            'WAYLAND_DISPLAY': 'wayland-0',  
            'XDG_RUNTIME_DIR': '/run/user/1000' 
        },
        output='screen'
    )

    # 把这四个任务打包在一起，同时扔给系统执行！
    return LaunchDescription([
        nav2_launch,
        mic_node,
        master_process,
        qt_process
    ])