cmake_minimum_required(VERSION 3.10)
# 给你的机甲视觉引擎起个霸气的名字
project(MUSE_Pi_Pro_3D_Radar)

# 设置 C++ 标准为 C++17 (ONNX Runtime 和现代 OpenCV 推荐)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 寻找 OpenCV 库 (包含核心运算、SGBM双目、以及 YOLOv8 必须的 DNN 模块)
find_package(OpenCV REQUIRED)

# 包含 OpenCV 头文件路径
include_directories(${OpenCV_INCLUDE_DIRS})

# ==========================================================
# 添加你的可执行文件 (编译 main.cpp)
# 如果你的文件名不是 main.cpp，请改成实际的名字
# ==========================================================
add_executable(ultimate_radar main.cpp)

# ==========================================================
# 核心链接环节：把双目视觉、ONNX 运行库、SpacemiT NPU 加速库缝合
# ==========================================================
target_link_libraries(ultimate_radar 
    ${OpenCV_LIBS} 
    onnxruntime         # 链接 ONNX 推理后端
    spacemit_ort_env    # 链接 进迭 K1 NPU 专属环境
)
file(GLOB SRCS "*.cc" "*.cpp")