#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <vector>
#include <cstdio>
#include <algorithm> // 用于中值排序
#include <cmath>     // 引入数学库处理 isinf
#include <iomanip>
#include <thread>
#include <cstdlib>
#include <filesystem>
#include <csignal>
#include <sys/prctl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include "yolov8.h"
#include "rvv_stereo_opt.h"

// 🟢 ROS 2 核心组件
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std;
using namespace cv;

namespace {
constexpr const char* kCameraDevice = "/dev/video20";
constexpr int kCameraWidth = 1280;
constexpr int kCameraHeight = 480;
constexpr int kCameraFps = 15;
constexpr int kProcessingFps = 5;
constexpr int kOpenAttempts = 3;
constexpr int kRuntimeFailureLimit = 3;

pid_t StartFrameServer(const char* argv0) {
    vector<filesystem::path> script_candidates;
    if (const char* configured = getenv("YOLO_FRAME_SERVER_SCRIPT")) {
        if (*configured) script_candidates.emplace_back(configured);
    }
    if (argv0 && *argv0) {
        script_candidates.push_back(
            filesystem::absolute(argv0).parent_path() / "yolo_frame_server.py"
        );
    }
    script_candidates.emplace_back(
        "/home/zyc/robot2/install/yolov8_ros2/lib/yolov8_ros2/yolo_frame_server.py"
    );
    script_candidates.emplace_back(
        "/home/zyc/robot2/src/yolov8_ros2/yolo_frame_server.py"
    );

    filesystem::path server_script;
    for (const auto& candidate : script_candidates) {
        if (filesystem::exists(candidate)) {
            server_script = candidate;
            break;
        }
    }
    if (server_script.empty()) {
        cerr << "【手机画面】未找到 yolo_frame_server.py，请检查功能包安装结果。" << endl;
        return -1;
    }

    const char* port_env = getenv("YOLO_FRAME_SERVER_PORT");
    const string port = (port_env && *port_env) ? port_env : "8080";
    pid_t child_pid = fork();
    if (child_pid < 0) {
        perror("【手机画面】创建服务器子进程失败");
        return -1;
    }

    if (child_pid == 0) {
        // 即使视觉节点被强制结束，也不会留下占用 8080 端口的孤儿进程。
        prctl(PR_SET_PDEATHSIG, SIGTERM);
        // 不把相机、ROS2 Socket 等父进程描述符带入 HTTP 服务器。
        for (int fd = 3; fd < 1024; ++fd) close(fd);
        execlp(
            "python3",
            "python3",
            "-u",
            server_script.c_str(),
            "--host",
            "0.0.0.0",
            "--port",
            port.c_str(),
            static_cast<char*>(nullptr)
        );
        perror("【手机画面】启动 Python 服务器失败");
        _exit(127);
    }

    this_thread::sleep_for(chrono::milliseconds(500));
    int status = 0;
    if (waitpid(child_pid, &status, WNOHANG) == child_pid) {
        cerr << "【手机画面】服务器未能启动，可能是端口 " << port
             << " 已被占用。视觉识别将继续运行。" << endl;
        return -1;
    }

    cout << "【手机画面】服务器已随视觉节点启动：http://0.0.0.0:"
         << port << "/" << endl;
    cout << "【手机画面】原始画面 /snapshot.jpg，识别画面 /debug.jpg" << endl;
    return child_pid;
}

void StopFrameServer(pid_t child_pid) {
    if (child_pid <= 0) return;

    kill(child_pid, SIGTERM);
    for (int i = 0; i < 10; ++i) {
        int status = 0;
        if (waitpid(child_pid, &status, WNOHANG) == child_pid) {
            cout << "【手机画面】服务器已随视觉节点关闭。" << endl;
            return;
        }
        this_thread::sleep_for(chrono::milliseconds(100));
    }

    kill(child_pid, SIGKILL);
    waitpid(child_pid, nullptr, 0);
    cout << "【手机画面】服务器已强制关闭。" << endl;
}

bool IsValidStereoFrame(const Mat& frame) {
    return !frame.empty()
        && frame.cols == kCameraWidth
        && frame.rows == kCameraHeight;
}

bool OpenStereoCamera(VideoCapture& cap, Mat& first_frame) {
    first_frame.release();

    for (int attempt = 1; attempt <= kOpenAttempts; ++attempt) {
        cap.release();
        cout << "【相机初始化】正在打开 " << kCameraDevice
             << "，第 " << attempt << "/" << kOpenAttempts << " 次尝试..." << endl;

        if (!cap.open(kCameraDevice, CAP_V4L2)) {
            cerr << "【相机初始化】设备打开失败，稍后重试。" << endl;
            this_thread::sleep_for(chrono::milliseconds(800));
            continue;
        }

        cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap.set(CAP_PROP_FRAME_WIDTH, kCameraWidth);
        cap.set(CAP_PROP_FRAME_HEIGHT, kCameraHeight);
        cap.set(CAP_PROP_FPS, kCameraFps);
        cap.set(CAP_PROP_BUFFERSIZE, 1);

        Mat candidate;
        bool read_ok = false;
        try {
            read_ok = cap.read(candidate);
        } catch (const cv::Exception& e) {
            cerr << "【相机初始化】读取首帧异常：" << e.what() << endl;
        }

        if (read_ok && IsValidStereoFrame(candidate)) {
            first_frame = candidate;
            cout << "【相机初始化】首帧读取成功："
                 << first_frame.cols << "x" << first_frame.rows
                 << "，MJPG，驱动帧率 " << cap.get(CAP_PROP_FPS) << " FPS。" << endl;
            return true;
        }

        cerr << "【相机初始化】首帧超时或尺寸异常，释放设备后重新打开。" << endl;
        cap.release();
        this_thread::sleep_for(chrono::milliseconds(800));
    }

    return false;
}
}  // namespace

int main(int argc, char** argv) {
    // OpenCV V4L2 默认单次 select 等待约 10 秒，缩短后可以更快触发自动重连。
    setenv("OPENCV_VIDEOIO_V4L_SELECT_TIMEOUT", "3", 0);

    // 🟢 1. 唤醒 ROS 2 节点
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("yolov8_sniper_node");
    
    // 🟢 2. 安装“嘴巴”：创建坐标发布器与图像发布器
    auto point_pub = node->create_publisher<geometry_msgs::msg::PointStamped>("/target_point", 10);
    auto image_pub = node->create_publisher<sensor_msgs::msg::Image>("/yolov8/debug_image", 10);

    cout << "\n==================================================" << endl;
    cout << "  K1 MUSE Pi Pro 双目视觉识别节点（ROS2 联动版）" << endl;
    cout << "==================================================" << endl;
    cout << "【题目一创新展示】RVV 加速双目视觉数据处理："
         << (stereo_rvv::RvvAvailable() ? "已启用" : "未启用") << endl;
    cout << "【说明】终端将每连续 30 帧统计一次平均耗时，便于拍摄对比。" << endl << endl;

    // 释放全部 8 核算力
    cv::setNumThreads(8);

    // ==========================================================
    // 光学烧录 & 数学降维 (保持完美的 320x240 物理精度)
    // ==========================================================
    Mat K1_orig = (Mat_<double>(3, 3) << 498.041273, 0., 322.919017, 0., 500.842780, 238.281081, 0., 0., 1.);
    Mat D1 = (Mat_<double>(1, 5) << 0.058602, 0.024552, 0., 0., 0.);
    Mat K2_orig = (Mat_<double>(3, 3) << 492.695619, 0., 322.558795, 0., 496.343841, 234.106179, 0., 0., 1.);
    Mat D2 = (Mat_<double>(1, 5) << 0.076096, -0.043065, 0., 0., 0.);
    Mat R = (Mat_<double>(3, 3) << 1.000000, 0.000069, 0.005549, -0.000071, 1.000000, 0.000398, -0.005549, -0.000398, 1.000000);
    Mat T = (Mat_<double>(3, 1) << -124.030072, 0.013520, 0.595863);

    Mat K1 = K1_orig.clone(); K1.at<double>(0, 0) /= 2.0; K1.at<double>(1, 1) /= 2.0; K1.at<double>(0, 2) /= 2.0; K1.at<double>(1, 2) /= 2.0;
    Mat K2 = K2_orig.clone(); K2.at<double>(0, 0) /= 2.0; K2.at<double>(1, 1) /= 2.0; K2.at<double>(0, 2) /= 2.0; K2.at<double>(1, 2) /= 2.0;

    Size small_size(320, 240);
    Mat R1, R2, P1, P2, Q_small;
    stereoRectify(K1, D1, K2, D2, small_size, R, T, R1, R2, P1, P2, Q_small, CALIB_ZERO_DISPARITY, 0);

    Mat map1x_small, map1y_small, map2x_small, map2y_small;
    initUndistortRectifyMap(K1, D1, R1, P1, small_size, CV_16SC2, map1x_small, map1y_small);
    initUndistortRectifyMap(K2, D2, R2, P2, small_size, CV_16SC2, map2x_small, map2y_small);

    // ==========================================================
    // 恢复满血 SGBM 引擎
    // ==========================================================
    int window_size = 5;
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(
        0, 128, window_size, 
        8 * 3 * window_size * window_size, 32 * 3 * window_size * window_size,
        2, 63, 10, 100, 32, StereoSGBM::MODE_SGBM_3WAY
    );

// ✅ 改成这样（绝对坐标，精准定位）：
string modelPath = "/home/zyc/robot2/src/yolov8_ros2/model/best.q.onnx";    cout << "【模型加载】正在加载 YOLOv8 目标识别模型：" << modelPath << endl;
    Mat dummy = Mat::zeros(240, 320, CV_8UC3);
    Yolov8_Fast_GetBoxes(dummy, modelPath); 

    Mat frame, left_gray, right_gray, left_small, right_small, rect_L, rect_R;
    VideoCapture cap;
    while (rclcpp::ok() && !OpenStereoCamera(cap, frame)) {
        cerr << "【相机初始化】本轮重试失败，2秒后继续，不需要手动重启节点。" << endl;
        if (rclcpp::ok()) rclcpp::spin_some(node);
        this_thread::sleep_for(chrono::seconds(2));
    }
    if (!rclcpp::ok()) {
        cap.release();
        rclcpp::shutdown();
        return -1;
    }

    cout << "【系统状态】已获得有效双目首帧，YOLOv8、SGBM 深度估计和 ROS2 网络正式启动。" << endl;

    Mat rect_L_color;
    int frame_count = 0;
    int consecutive_camera_failures = 0;
    const string phone_raw_tmp = "/tmp/yolov8_raw_latest.tmp.jpg";
    const string phone_raw_path = "/tmp/yolov8_raw_latest.jpg";
    const string phone_debug_tmp = "/tmp/yolov8_debug_latest.tmp.jpg";
    const string phone_debug_path = "/tmp/yolov8_debug_latest.jpg";
    vector<int> phone_jpeg_params = {IMWRITE_JPEG_QUALITY, 82};
    std::remove(phone_raw_tmp.c_str());
    std::remove(phone_raw_path.c_str());
    std::remove(phone_debug_tmp.c_str());
    std::remove(phone_debug_path.c_str());
    pid_t frame_server_pid = StartFrameServer(argv[0]);
    int rvv_window_count = 0;
    double rvv_scalar_sum = 0.0;
    double rvv_vector_sum = 0.0;
    int rvv_active_pixels = 0;
    int rvv_mismatch_pixels = 0;
    auto prev_time = chrono::high_resolution_clock::now();

    // 🟢 核心改动：把 while(true) 替换为 ROS 2 的存活状态判定
    while (rclcpp::ok()) {
        const auto loop_started_at = chrono::steady_clock::now();
        if (frame.empty()) {
            bool read_ok = false;
            try {
                read_ok = cap.read(frame);
            } catch (const cv::Exception& e) {
                cerr << "【相机运行】读取异常：" << e.what() << endl;
            }

            if (!read_ok || !IsValidStereoFrame(frame)) {
                frame.release();
                ++consecutive_camera_failures;
                cerr << "【相机运行】取帧失败 " << consecutive_camera_failures
                     << "/" << kRuntimeFailureLimit << endl;

                if (consecutive_camera_failures >= kRuntimeFailureLimit) {
                    cerr << "【相机运行】连续取帧失败，正在自动重启双目相机..." << endl;
                    while (rclcpp::ok() && !OpenStereoCamera(cap, frame)) {
                        cerr << "【相机运行】重连失败，2秒后继续尝试。" << endl;
                        if (rclcpp::ok()) rclcpp::spin_some(node);
                        this_thread::sleep_for(chrono::seconds(2));
                    }
                    if (!rclcpp::ok()) break;
                    cout << "【相机运行】双目相机已自动恢复。" << endl;
                    consecutive_camera_failures = 0;
                } else {
                    if (rclcpp::ok()) rclcpp::spin_some(node);
                    continue;
                }
            }
        }

        if (!IsValidStereoFrame(frame)) {
            frame.release();
            continue;
        }
        consecutive_camera_failures = 0;
        frame_count++;

        if (frame.channels() == 3) {
            cvtColor(frame(Rect(0, 0, 640, 480)), left_gray, COLOR_BGR2GRAY);
            cvtColor(frame(Rect(640, 0, 640, 480)), right_gray, COLOR_BGR2GRAY);
        } else {
            left_gray = frame(Rect(0, 0, 640, 480)); right_gray = frame(Rect(640, 0, 640, 480));
        }
        
        // 极速缩小
        resize(left_gray, left_small, Size(320, 240), 0, 0, INTER_NEAREST); 
        resize(right_gray, right_small, Size(320, 240), 0, 0, INTER_NEAREST);
        remap(left_small, rect_L, map1x_small, map1y_small, INTER_NEAREST);
        remap(right_small, rect_R, map2x_small, map2y_small, INTER_NEAREST);
        cvtColor(rect_L, rect_L_color, COLOR_GRAY2BGR); 

        Mat raw_for_phone = rect_L_color.clone();
        if (frame_count % 3 == 0) {
            try {
                imwrite(phone_raw_tmp, raw_for_phone, phone_jpeg_params);
                std::rename(phone_raw_tmp.c_str(), phone_raw_path.c_str());
            } catch (...) {
            }
        }

        auto rvv_report = stereo_rvv::BenchmarkStereoPreprocess(rect_L, rect_R, 1, 18);
        rvv_window_count++;
        rvv_scalar_sum += rvv_report.scalar_ms;
        rvv_vector_sum += rvv_report.rvv_ms;
        rvv_active_pixels = rvv_report.active_pixels;
        rvv_mismatch_pixels += rvv_report.mismatch_pixels;

        if (rvv_window_count >= 30) {
            double scalar_avg = rvv_scalar_sum / rvv_window_count;
            double rvv_avg = rvv_vector_sum / rvv_window_count;
            double speedup = (rvv_avg > 0.0) ? scalar_avg / rvv_avg : 0.0;

            cout << fixed << setprecision(4);
            cout << "\n\n========== 【RVV 双目相机数据处理优化展示】 ==========" << endl;
            cout << "【统计方式】连续 " << rvv_window_count << " 帧实时统计平均耗时" << endl;
            cout << "【输入画面】双目原始画面 1280x480，左目/右目各 640x480" << endl;
            cout << "【处理画面】校正后双目灰度图 " << rect_L.cols << "x" << rect_L.rows
                 << "，像素数量 " << rvv_report.pixels << endl;
            cout << "【处理任务】左右图像像素差计算 + 阈值提取，用于目标区域和深度计算前的数据准备" << endl;
            cout << "【普通标量循环】平均耗时：" << scalar_avg << " ms / 帧" << endl;
            cout << "【RVV 向量优化】平均耗时：" << rvv_avg << " ms / 帧" << endl;
            cout << "【加速效果】RVV 相比普通标量约提升：" << speedup << " 倍" << endl;
            cout << "【结果校验】两种算法输出不一致像素数：" << rvv_mismatch_pixels
                 << "（0 表示结果一致）" << endl;
            cout << "【有效像素】当前帧提取到的变化/候选像素：" << rvv_active_pixels << endl;
            cout << "【对应题目】复杂信号处理 + 程序优化 + RISC-V RVV 向量扩展应用" << endl;
            cout << "====================================================\n" << endl;

            rvv_window_count = 0;
            rvv_scalar_sum = 0.0;
            rvv_vector_sum = 0.0;
            rvv_mismatch_pixels = 0;
        }

        std::vector<Object> detected_targets = Yolov8_Fast_GetBoxes(rect_L_color, modelPath);

        int min_y = 240, max_y = 0;
        std::vector<Object> valid_targets;
        
        for (const auto& t : detected_targets) {
            if (t.score >= 0.6f) {
                valid_targets.push_back(t);
                min_y = min(min_y, max(0, (int)t.y1));
                max_y = max(max_y, min(239, (int)t.y2));
            }
        }

        // 🟢 准备 ROS 2 消息发送容器
        geometry_msgs::msg::PointStamped best_target_msg;
        best_target_msg.header.frame_id = "camera_color_optical_frame";
        bool has_target_to_pub = false;
        float highest_score = 0.0f;
        int best_cx = -1;
        int best_cy = -1;
        double best_depth_m = 0.0;

        if (!valid_targets.empty()) {
            min_y = max(0, min_y - 20);
            max_y = min(239, max_y + 20);
            int roi_height = max_y - min_y + 1;
            
            Rect dynamic_roi(0, min_y, 320, roi_height);
            Mat disp16_roi;
            
            sgbm->compute(rect_L(dynamic_roi), rect_R(dynamic_roi), disp16_roi);

            for (const auto& target : valid_targets) {
                int x1 = max(0, (int)target.x1); int y1 = max(0, (int)target.y1);
                int x2 = min(319, (int)target.x2); int y2 = min(239, (int)target.y2);
                int box_cx = (x1 + x2) / 2; int box_cy = (y1 + y2) / 2;

                int local_cy = box_cy - min_y; 

                int radius = 10; 
                int rx = max(0, box_cx - radius);
                int ry_local = max(0, local_cy - radius);
                int rw = min(319 - rx, radius * 2);
                int rh_local = min(roi_height - 1 - ry_local, radius * 2);
                
                std::vector<float> valid_disps;
                for(int r = ry_local; r < ry_local + rh_local; r++) {
                    for(int c = rx; c < rx + rw; c++) {
                        short d_short = disp16_roi.at<short>(r, c);
                        float d = d_short / 16.0f;
                        if(d > 2.0f) valid_disps.push_back(d); 
                    }
                }

                double Z = 0;
                if (valid_disps.size() > 0) {
                    std::nth_element(valid_disps.begin(), valid_disps.begin() + valid_disps.size()/2, valid_disps.end());
                    float median_disp = valid_disps[valid_disps.size()/2];
                    
                    double W = Q_small.at<double>(3, 2) * median_disp + Q_small.at<double>(3, 3);
                    Z = Q_small.at<double>(2, 3) / W; 
                }

                int ry_global = ry_local + min_y; 
                rectangle(rect_L_color, Point(x1, y1), Point(x2, y2), Scalar(0, 0, 255), 2);
                rectangle(rect_L_color, Rect(rx, ry_global, rw, rh_local), Scalar(255, 0, 0), 1);
                drawMarker(rect_L_color, Point(box_cx, box_cy), Scalar(0, 255, 0), MARKER_CROSS, 10, 2);
                
                char label_buf[64];
                if (Z > 0 && !std::isinf(Z) && std::abs(Z) < 800) {
                    sprintf(label_buf, "Bottle(%.0f%%): %dmm", target.score * 100, (int)std::abs(Z));

                    // 🟢 计算全局 3D 坐标并选出置信度最高的目标发给机械臂
                    if (target.score > highest_score) {
                        highest_score = target.score;
                        
                        // 运用 Q 矩阵对应的相机内参推算真实物理空间的 X, Y
                        double cx = -Q_small.at<double>(0, 3);
                        double cy = -Q_small.at<double>(1, 3);
                        double f = Q_small.at<double>(2, 3);

                        double real_x = (box_cx - cx) * Z / f;
                        double real_y = (box_cy - cy) * Z / f;

                        // 装载 ROS 2 数据 (必须将毫米 mm 转换为米 m)
                        best_target_msg.point.x = real_x / 1000.0;
                        best_target_msg.point.y = real_y / 1000.0;
                        best_target_msg.point.z = Z / 1000.0;
                        best_cx = box_cx;
                        best_cy = box_cy;
                        best_depth_m = Z / 1000.0;
                        has_target_to_pub = true;
                    }
                } else {
                    sprintf(label_buf, "Bottle(%.0f%%): Blind", target.score * 100);
                }
                putText(rect_L_color, label_buf, Point(x1, max(10, y1 - 5)), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 2);
            }
        }

        // 🟢 广播目标三维坐标
        if (has_target_to_pub) {
            best_target_msg.header.stamp = node->now();
            point_pub->publish(best_target_msg);
            if (frame_count % 15 == 0) {
                cout << fixed << setprecision(3);
                cout << "\n【目标定位】中心点=(" << best_cx
                     << ", " << best_cy
                     << ")，深度=" << best_depth_m << " m"
                     << "，已发布到 ROS2 话题 /target_point，供机械臂抓取使用。" << endl;
            }
        }

        // 视觉推理限制在 5 FPS，瓶子识别仍然足够实时，同时为 Nav2、
        // 语音和机械臂预留 CPU，避免比赛导航期间生命周期心跳丢失。
        const auto next_frame_at = loop_started_at
            + chrono::milliseconds(1000 / kProcessingFps);
        const auto before_deadline = chrono::steady_clock::now();
        if (before_deadline < next_frame_at) {
            this_thread::sleep_until(next_frame_at);
        }

        auto curr_time = chrono::high_resolution_clock::now();
        chrono::duration<float> elapsed = curr_time - prev_time;
        float fps = 1.0f / elapsed.count(); 
        prev_time = curr_time;

        if (frame_count > 2) {
             printf("\r【ROS2视觉节点】实时帧率: %5.1f FPS | 检测目标数: %lu        ", fps, valid_targets.size()); 
             fflush(stdout); 
        }

        if (frame_count % 2 == 0) {
            putText(rect_L_color, "FPS: " + to_string((int)fps), Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
            try {
                imwrite(phone_debug_tmp, rect_L_color, phone_jpeg_params);
                std::rename(phone_debug_tmp.c_str(), phone_debug_path.c_str());
            } catch (...) {
            }
            
            // 🟢 通过 ROS 2 广播实时画面图像 (用 rqt_image_view 可以远程观看)
            sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rect_L_color).toImageMsg();
            image_pub->publish(*img_msg);
        }

        // 🟢 处理 ROS 2 底层通讯回调
        frame.release();
        if (rclcpp::ok()) rclcpp::spin_some(node);
    }
    
    StopFrameServer(frame_server_pid);
    cap.release(); 
    if (rclcpp::ok()) rclcpp::shutdown(); // 🟢 优雅地关闭 ROS 2
    cout.flush();
    cerr.flush();
    // 部分厂商 ONNX/OpenCV 动态库在静态析构阶段会重复释放资源。
    // 关键资源已经显式关闭，直接结束进程可避免退出时的假性段错误。
    std::_Exit(0);
}
