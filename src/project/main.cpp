#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

using namespace std;
using namespace cv;

int main() {
    cout << "\n🚀 唤醒【机甲视觉封存版】(内参数学降维 | 绝对物理精度 | 极限 FPS)..." << endl;

    cv::setNumThreads(8);

    // ==========================================================
    // 1. 核心光学数据烧录 (640x480 原始数据)
    // ==========================================================
    Mat K1_orig = (Mat_<double>(3, 3) << 498.041273, 0., 322.919017, 0., 500.842780, 238.281081, 0., 0., 1.);
    Mat D1 = (Mat_<double>(1, 5) << 0.058602, 0.024552, 0., 0., 0.);
    Mat K2_orig = (Mat_<double>(3, 3) << 492.695619, 0., 322.558795, 0., 496.343841, 234.106179, 0., 0., 1.);
    Mat D2 = (Mat_<double>(1, 5) << 0.076096, -0.043065, 0., 0., 0.);
    Mat R = (Mat_<double>(3, 3) << 1.000000, 0.000069, 0.005549, -0.000071, 1.000000, 0.000398, -0.005549, -0.000398, 1.000000);
    Mat T = (Mat_<double>(3, 1) << -124.030072, 0.013520, 0.595863);

    // ==========================================================
    // 💥 终极数学降维：将光学内参直接缩小一半，适配 320x240！
    // ==========================================================
    Mat K1 = K1_orig.clone();
    K1.at<double>(0, 0) /= 2.0; // fx 减半
    K1.at<double>(1, 1) /= 2.0; // fy 减半
    K1.at<double>(0, 2) /= 2.0; // cx 减半
    K1.at<double>(1, 2) /= 2.0; // cy 减半

    Mat K2 = K2_orig.clone();
    K2.at<double>(0, 0) /= 2.0; 
    K2.at<double>(1, 1) /= 2.0; 
    K2.at<double>(0, 2) /= 2.0; 
    K2.at<double>(1, 2) /= 2.0; 

    Size small_size(320, 240); // 目标引擎尺寸
    Mat R1, R2, P1, P2, Q_small;
    // 使用降维后的参数生成全新的 Q 矩阵
    stereoRectify(K1, D1, K2, D2, small_size, R, T, R1, R2, P1, P2, Q_small, CALIB_ZERO_DISPARITY, 0);

    Mat map1x_small, map1y_small, map2x_small, map2y_small;
    // 直接生成 320x240 专属的轻量级畸变表
    initUndistortRectifyMap(K1, D1, R1, P1, small_size, CV_16SC2, map1x_small, map1y_small);
    initUndistortRectifyMap(K2, D2, R2, P2, small_size, CV_16SC2, map2x_small, map2y_small);

    // ==========================================================
    // 2. 构造 SGBM 引擎 (维持 96 层大视角，确保看清近处)
    // ==========================================================
    int window_size = 5;
    int num_disp = 96; 
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(
        0, num_disp, window_size,
        8 * 3 * window_size * window_size, 32 * 3 * window_size * window_size,
        2, 63, 10, 100, 32, StereoSGBM::MODE_SGBM_3WAY
    );

    // ==========================================================
    // 3. 打开相机
    // ==========================================================
    VideoCapture cap(20, CAP_V4L2);
    if (!cap.isOpened()) return -1;

    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G')); 
    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(CAP_PROP_FPS, 30); 

    cout << "✅ 引擎重构完毕，内参降维成功，准备极限测速..." << endl;
    cout << "--------------------------------------------------------" << endl;

    Mat frame, left_gray, right_gray, left_small, right_small, rect_L, rect_R, disp16, disp_float;
    int frame_count = 0;
    auto prev_time = chrono::high_resolution_clock::now();

    namedWindow("MUSE Pi Pro - 3D Radar", WINDOW_AUTOSIZE);

    while (true) {
        try { cap >> frame; } catch (...) { continue; }
        if (frame.empty() || frame.cols != 1280) continue;

        frame_count++;

        cvtColor(frame(Rect(0, 0, 640, 480)), left_gray, COLOR_BGR2GRAY);
        cvtColor(frame(Rect(640, 0, 640, 480)), right_gray, COLOR_BGR2GRAY);

        // 💥 关键性能飞跃点：先缩小，后极速校正！
        resize(left_gray, left_small, Size(320, 240));
        resize(right_gray, right_small, Size(320, 240));

        remap(left_small, rect_L, map1x_small, map1y_small, INTER_LINEAR);
        remap(right_small, rect_R, map2x_small, map2y_small, INTER_LINEAR);

        // 核心计算
        sgbm->compute(rect_L, rect_R, disp16);
        disp16.convertTo(disp_float, CV_32F, 1.0 / 16.0);

        auto curr_time = chrono::high_resolution_clock::now();
        chrono::duration<float> elapsed = curr_time - prev_time;
        float fps = 1.0f / elapsed.count();
        prev_time = curr_time;

        // 💥 最纯粹的 3D 解算：再也不需要手动乘 2 了，因为 Q_small 已经是降维后的完全体！
        int cx = 160, cy = 120; 
        float center_disp = disp_float.at<float>(cy, cx);
        double Z = 0;

        if (center_disp > 0.1) {
            double W = Q_small.at<double>(3, 2) * center_disp + Q_small.at<double>(3, 3);
            Z = Q_small.at<double>(2, 3) / W; 
        }

        if (frame_count > 2) {
            if (Z <= 0 || isinf(Z) || abs(Z) > 10000) {
                printf("\r[引擎转速: %5.1f FPS] 中心点: 盲区        ", fps);
            } else {
                printf("\r[引擎转速: %5.1f FPS] 中心点距离: %6.1f 毫米        ", fps, abs(Z));
            }
            fflush(stdout); 
        }

        // SSH 传输 (每两帧一次)
        if (frame_count % 2 == 0) {
            Mat disp_norm, disp_color;
            normalize(disp_float, disp_norm, 0, 255, NORM_MINMAX, CV_8U);
            applyColorMap(disp_norm, disp_color, COLORMAP_JET);
            disp_color.setTo(Scalar(0, 0, 0), disp_float <= 0); 

            Mat rect_L_color;
            cvtColor(rect_L, rect_L_color, COLOR_GRAY2BGR);
            drawMarker(rect_L_color, Point(cx, cy), Scalar(0, 0, 255), MARKER_CROSS, 15, 2);
            putText(rect_L_color, "FPS: " + to_string((int)fps), Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);

            Mat display_img;
            hconcat(rect_L_color, disp_color, display_img);
            imshow("MUSE Pi Pro - 3D Radar", display_img);
        }

        if (waitKey(1) == 'q') break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}