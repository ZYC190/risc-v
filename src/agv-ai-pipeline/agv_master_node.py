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
from rclpy.executors import MultiThreadedExecutor

import subprocess
import re

from agv_common_class import  NavigationClient, AngleSubscriber, OdomSubscriber, RotateRobot, AngleNavGoalPoseSend

# ours
# 大模型
# from spacemit_llm import LLMModel, FCModel

# 音频
from spacemit_audio import find_audio_card, find_playback_card
from spacemit_audio import play_wav_non_blocking, play_wav, play_audio_in_thread
from spacemit_audio import ASRModel
from spacemit_audio.record_loop import RecAudioDB
asr_model = ASRModel()
from tools.agv import FuncControllerNode, execute_command


record_device = find_audio_card()
play_device = f'plughw:{find_playback_card()},0'


def run_executor(executor):
    executor.spin()  # 运行 ROS 2 事件循环（不会阻塞主线程）

def main():
    rclpy.init()

    func_controller_node = FuncControllerNode() # 总控制
    angle_sub_node = AngleSubscriber()  # 声源定位信息订阅
    odom_sub_node = OdomSubscriber()    # 里程计信息订阅
    node_rotate = RotateRobot(odom_sub_node, angle_sub_node)  # 传递订阅节点的引用

    # 多线程执行的管理器
    executor = MultiThreadedExecutor()
    executor.add_node(func_controller_node)
    executor.add_node(angle_sub_node)
    executor.add_node(odom_sub_node)
    executor.add_node(node_rotate)

    executor_thread = threading.Thread(target=run_executor, args=(executor,), daemon=True)
    executor_thread.start()

    # 录音功能类初始化

    rec_audio = RecAudioDB(sld=1, min_db=4000, max_time=30, rate=16000, device_index=record_device)

    # wav_file_path = "tools/feedback_voice/dengdaihuanxing.wav"
    # play_wav(wav_file_path, device=play_device)
    try:
        while rclpy.ok():

            # 播放唤醒反馈
            # wav_file_path = "tools/feedback_voice/awake.wav"
            # play_wav(wav_file_path, device=play_device)

            print("等待唤醒...")
            angle_sub_node.ready_to_wait_event.set()
            angle_sub_node.trigger_event.wait()
            angle_sub_node.trigger_event.clear()

            node_rotate.rotate_to_angle()  # 触发旋转

            while rclpy.ok():

                angle_sub_node.ready_to_wait_event.clear()
            
                # 开始录制用户声音
                # b = input()
                print("开始录音")
                audio_ret = rec_audio.record_audio()

                if rec_audio.exit_mode == 0:
                    # 语音转文字
                    print("开始语音转文字----------------------")
                    text = asr_model.generate(audio_ret)
                    print('user: ', text)

                    # 模糊匹配
                    ret, func_name = execute_command(text, func_controller_node.function_dict_zh)
                    if ret:
                        print(f"调用的函数名: {func_name}")
                        func_controller_node.robot_stop_move()

                        if func_name == 'rotate_in_place':
                            print("原地旋转")
                            # wav_file_path = "tools/feedback_voice/haode.wav"
                            # play_wav_non_blocking(wav_file_path, device=play_device)

                        if func_name == 'follow_me':
                            print("开始跟随..............")
                            # wav_file_path = "tools/feedback_voice/haodekaishigensui.wav"
                            # play_wav_non_blocking(wav_file_path, device=play_device)

                        func_controller_node.function_dict[func_name]()

                        if func_name == 'move_away':
                            print("..............")
                            # wav_file_path = "tools/feedback_voice/zaidengzhe.wav"
                            # play_wav(wav_file_path, device=play_device)


                    else:
                        print("未匹配到函数......................")
                        # wav_file_path = "tools/feedback_voice/meitingqingchu.wav"
                        # play_wav(wav_file_path, device=play_device)


                else:
                    print(f"超过设定时间未检测到人声, 进入唤醒模式！")
                    # wav_file_path = "tools/feedback_voice/jinruhuanxing.wav"
                    # play_wav(wav_file_path, device=play_device)
                    break

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
