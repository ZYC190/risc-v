#include "rvv_stereo_opt.h"

#include <algorithm>
#include <chrono>
#include <vector>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace stereo_rvv {
namespace {

void ScalarAbsDiffThreshold(
    const uint8_t* left,
    const uint8_t* right,
    uint8_t* mask,
    int pixels,
    uint8_t threshold) {
    for (int i = 0; i < pixels; ++i) {
        int diff = static_cast<int>(left[i]) - static_cast<int>(right[i]);
        if (diff < 0) diff = -diff;
        mask[i] = diff > threshold ? 255 : 0;
    }
}

#if defined(__riscv_vector)
void RvvAbsDiffThreshold(
    const uint8_t* left,
    const uint8_t* right,
    uint8_t* mask,
    int pixels,
    uint8_t threshold) {
    int i = 0;
    while (i < pixels) {
        size_t vl = __riscv_vsetvl_e8m8(static_cast<size_t>(pixels - i));
        vuint8m8_t v_left = __riscv_vle8_v_u8m8(left + i, vl);
        vuint8m8_t v_right = __riscv_vle8_v_u8m8(right + i, vl);
        vuint8m8_t v_max = __riscv_vmaxu_vv_u8m8(v_left, v_right, vl);
        vuint8m8_t v_min = __riscv_vminu_vv_u8m8(v_left, v_right, vl);
        vuint8m8_t v_diff = __riscv_vsub_vv_u8m8(v_max, v_min, vl);
        vbool1_t v_mask = __riscv_vmsgtu_vx_u8m8_b1(v_diff, threshold, vl);
        vuint8m8_t v_zero = __riscv_vmv_v_x_u8m8(0, vl);
        vuint8m8_t v_out = __riscv_vmerge_vxm_u8m8(v_zero, 255, v_mask, vl);
        __riscv_vse8_v_u8m8(mask + i, v_out, vl);
        i += static_cast<int>(vl);
    }
}
#else
void RvvAbsDiffThreshold(
    const uint8_t* left,
    const uint8_t* right,
    uint8_t* mask,
    int pixels,
    uint8_t threshold) {
    ScalarAbsDiffThreshold(left, right, mask, pixels, threshold);
}
#endif

double MsBetween(
    const std::chrono::high_resolution_clock::time_point& start,
    const std::chrono::high_resolution_clock::time_point& end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

}  // namespace

bool RvvAvailable() {
#if defined(__riscv_vector)
    return true;
#else
    return false;
#endif
}

StereoRvvReport BenchmarkStereoPreprocess(
    const cv::Mat& left_gray,
    const cv::Mat& right_gray,
    int loops,
    uint8_t threshold) {
    StereoRvvReport report;
    if (left_gray.empty() || right_gray.empty() ||
        left_gray.size() != right_gray.size() ||
        left_gray.type() != CV_8UC1 || right_gray.type() != CV_8UC1) {
        return report;
    }

    cv::Mat left = left_gray.isContinuous() ? left_gray : left_gray.clone();
    cv::Mat right = right_gray.isContinuous() ? right_gray : right_gray.clone();

    report.width = left.cols;
    report.height = left.rows;
    report.pixels = left.cols * left.rows;
    report.loops = std::max(1, loops);
    report.rvv_enabled = RvvAvailable();

    std::vector<uint8_t> scalar_mask(report.pixels);
    std::vector<uint8_t> rvv_mask(report.pixels);

    const uint8_t* left_ptr = left.ptr<uint8_t>(0);
    const uint8_t* right_ptr = right.ptr<uint8_t>(0);

    auto scalar_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < report.loops; ++i) {
        ScalarAbsDiffThreshold(left_ptr, right_ptr, scalar_mask.data(), report.pixels, threshold);
    }
    auto scalar_end = std::chrono::high_resolution_clock::now();

    auto rvv_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < report.loops; ++i) {
        RvvAbsDiffThreshold(left_ptr, right_ptr, rvv_mask.data(), report.pixels, threshold);
    }
    auto rvv_end = std::chrono::high_resolution_clock::now();

    report.scalar_ms = MsBetween(scalar_start, scalar_end) / report.loops;
    report.rvv_ms = MsBetween(rvv_start, rvv_end) / report.loops;
    if (report.rvv_ms > 0.0) {
        report.speedup = report.scalar_ms / report.rvv_ms;
    }
    report.active_pixels = static_cast<int>(std::count(rvv_mask.begin(), rvv_mask.end(), 255));
    report.mismatch_pixels = 0;
    for (int i = 0; i < report.pixels; ++i) {
        if (scalar_mask[i] != rvv_mask[i]) {
            ++report.mismatch_pixels;
        }
    }

    return report;
}

}  // namespace stereo_rvv
