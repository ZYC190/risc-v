// ------------------------
// main.cpp
// put the file in the project root directory
// ~/project/main.cpp
// ------------------------
#include <chrono>
#include <csignal>
#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
std::atomic<bool> running(true);

void signal_handler(int signum) {
    std::cout << "\nCaught signal " << signum << ", exiting...\n";
    running = false;  // 设置标志，退出 while 循环
}
int main(int argc, char **argv){
    signal(SIGINT, signal_handler);  // 捕获 Ctrl-C 信号
    int video_idx = std::stoi(argv[1]);
    std::string gst_pipeline = "v4l2src device=/dev/video" + std::to_string(video_idx) + " io-mode=2 ! " +
        "video/x-raw,format=YUY2,width=2560,height=720,framerate=60/1 ! " +
        "appsink";

    cv::VideoCapture cap(gst_pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera" << std::endl;
        return -1;
    }
    auto t0 = std::chrono::steady_clock::now();
    cv::Mat frame;
    int frame_id = 0;
    while(running && cap.grab()){
        cap.retrieve(frame);
        frame_id += 1;
        if(frame_id % 100 == 0){
            auto t1 = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
            printf("FPS %.2f Process \r\n", 100 / (duration / 1000000.0));
            t0 = t1;
        }
    }
    cap.release();
    return 0;
}
