cmake_minimum_required(VERSION 3.10)
# 💥 修改点 1：包名统一改为刚才 package.xml 里声明的名字
project(yolov8_ros2) 

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE RELEASE)

# ==========================================================
# 🟢 新增的 ROS 2 核心依赖
# ==========================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

file(GLOB SRCS "*.cc" "*.cpp")

# ==========================================================
# 💥 (完全保留您的配置)：强行指定手工编译的满血版 OpenCV 路径
# ==========================================================
set(OpenCV_DIR "/usr/local/lib/cmake/opencv4") 
find_package(OpenCV 4 REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# 手工开启 OpenMP 加速标志
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

# OpenCL
find_package(OpenCL QUIET)
if(OpenCL_FOUND)
    message(STATUS "OpenCL found: ${OpenCL_VERSION}")
    message(STATUS "  Includes: ${OpenCL_INCLUDE_DIRS}")
    message(STATUS "  Libraries: ${OpenCL_LIBRARIES}")
    # ⚠️ 警报：这个相对路径 ../../../../ 可能会因为文件搬家而失效
    # 如果编译报错找不到这两个 .cc 文件，您需要改成绝对路径！
    set(OPENCL_SOURCES
        ../../../../third-party/opencl/yolo-process/Remap.cc
        ../../../../third-party/opencl/yolo-process/utils.cc
    )
    add_definitions(-DUSE_OPENCL)
else()
    message(STATUS "OpenCL not found - building without OpenCL support")
    set(OPENCL_SOURCES "")
endif()

# ONNXRUNTIME
set(ONNXRUNTIME_INCLUDE_DIR "$ENV{HOME}/code/test_lib/spacemit-ort.riscv64.2.0.1/include")
set(ONNXRUNTIME_LIB_DIR "$ENV{HOME}/code/test_lib/spacemit-ort.riscv64.2.0.1/lib")
link_directories(${ONNXRUNTIME_LIB_DIR})

# 💥 修改点 2：将可执行文件命名为 yolov8_node，方便在 ROS 2 里调用
set(NODE_NAME "yolov8_node")
add_executable(${NODE_NAME} ${SRCS} ${OPENCL_SOURCES})

# ==========================================================
# 💥 (完全保留您的配置)：强制优先包含手工版 OpenCV 的头文件路径
# ==========================================================
target_include_directories(${NODE_NAME} PRIVATE /usr/local/include/opencv4)
target_include_directories(${NODE_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(${NODE_NAME} PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})

if(OpenCL_FOUND)
    target_include_directories(${NODE_NAME} PRIVATE ${OpenCL_INCLUDE_DIRS})
    target_include_directories(${NODE_NAME} PRIVATE ../../../../third-party/opencl/yolo-process/)
endif()

# ==========================================================
# 💥 (完全保留您的配置)：强制链接手工版 OpenCV 的库文件路径
# ==========================================================
target_link_directories(${NODE_NAME} PRIVATE /usr/local/lib)
target_link_libraries(${NODE_NAME} ${OpenCV_LIBS})
target_link_libraries(${NODE_NAME} onnxruntime spacemit_ep)

if(OpenCL_FOUND)
    target_link_libraries(${NODE_NAME} ${OpenCL_LIBRARIES} OpenCL::OpenCL gbm)
endif()

add_definitions(-D__fp16=_Float16)

# ==========================================================
# 🟢 新增的 ROS 2 链接与安装规则 (核心包装)
# ==========================================================
# 给这个节点挂载 ROS 2 的通讯库
ament_target_dependencies(${NODE_NAME}
    rclcpp
    sensor_msgs
    cv_bridge
)

# 安装规则：把编译好的节点放入机甲的启动目录
install(TARGETS ${NODE_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

# 安装规则：把模型文件夹一并带上，防止运行时找不到 best.q.onnx
install(DIRECTORY model/
    DESTINATION share/${PROJECT_NAME}/model
)

ament_package()