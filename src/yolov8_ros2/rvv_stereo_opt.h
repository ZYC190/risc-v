#pragma once

#include <cstdint>
#include <opencv2/core.hpp>

namespace stereo_rvv {

struct StereoRvvReport {
    int width = 0;
    int height = 0;
    int pixels = 0;
    int loops = 0;
    int active_pixels = 0;
    int mismatch_pixels = 0;
    double scalar_ms = 0.0;
    double rvv_ms = 0.0;
    double speedup = 0.0;
    bool rvv_enabled = false;
};

bool RvvAvailable();
StereoRvvReport BenchmarkStereoPreprocess(
    const cv::Mat& left_gray,
    const cv::Mat& right_gray,
    int loops = 30,
    uint8_t threshold = 18);

}  // namespace stereo_rvv
