#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>  // 增加控制速度的头文件
#include <riscv_vector.h>               // K1 芯片的 RVV 矢量指令头文件
#include <chrono>
#include <vector>
#include <cmath>

using std::placeholders::_1;

class RvvLidarProcessor : public rclcpp::Node {
public:
    RvvLidarProcessor() : Node("rvv_lidar_processor") {
        // 1. 订阅雷达数据 (感受器)
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(), std::bind(&RvvLidarProcessor::scan_callback, this, _1));
        
        // 2. 发布速度指令 (效应器：随时准备抢夺底盘控制权)
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        RCLCPP_INFO(this->get_logger(), "🛡️ [安全冗余系统] RVV 脑干反射节点已上线！正在提供微秒级前向碰撞保护...");
        tables_initialized_ = false;
    }

private:
    std::vector<float> cos_table_; 
    std::vector<float> sin_table_;
    bool tables_initialized_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // ⚡ 核心科技：进迭时空 K1 专属 RVV 1.0 矢量同时计算 X 和 Y
    void compute_xy_rvv(const float* ranges, const float* cos_angles, const float* sin_angles, float* x_out, float* y_out, size_t n) {
        size_t vl; 
        for (; n > 0; n -= vl, ranges += vl, cos_angles += vl, sin_angles += vl, x_out += vl, y_out += vl) {
            vl = __riscv_vsetvl_e32m1(n); 
            
            // 批量加载 距离、Cos表、Sin表
            vfloat32m1_t vranges = __riscv_vle32_v_f32m1(ranges, vl);
            vfloat32m1_t vcos = __riscv_vle32_v_f32m1(cos_angles, vl);
            vfloat32m1_t vsin = __riscv_vle32_v_f32m1(sin_angles, vl);
            
            // 批量执行乘法
            vfloat32m1_t vx = __riscv_vfmul_vv_f32m1(vranges, vcos, vl);
            vfloat32m1_t vy = __riscv_vfmul_vv_f32m1(vranges, vsin, vl);
            
            // 批量存回内存
            __riscv_vse32_v_f32m1(x_out, vx, vl);
            __riscv_vse32_v_f32m1(y_out, vy, vl);
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        size_t n = msg->ranges.size();
        if (n == 0) return;

        // 初始化三角函数查表 (全生命周期只算一次，极致优化)
        if (!tables_initialized_) {
            cos_table_.resize(n);
            sin_table_.resize(n);
            for (size_t i = 0; i < n; i++) {
                float angle = msg->angle_min + i * msg->angle_increment;
                cos_table_[i] = std::cos(angle);
                sin_table_[i] = std::sin(angle);
            }
            tables_initialized_ = true;
        }

        std::vector<float> x_out(n, 0.0f);
        std::vector<float> y_out(n, 0.0f);

        // --- ⚔️ 脑干反射开始 ⚔️ ---
        auto start_rvv = std::chrono::high_resolution_clock::now();
        
        // 1. 瞬间完成所有极坐标到直角坐标的转换！
        compute_xy_rvv(msg->ranges.data(), cos_table_.data(), sin_table_.data(), x_out.data(), y_out.data(), n);
        
        auto end_rvv = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::micro> us_rvv = end_rvv - start_rvv;

        // 2. 危险检测：扫描刚刚算出来的 X 和 Y
        bool danger_detected = false;
        for (size_t i = 0; i < n; i++) {
            float x = x_out[i];
            float y = y_out[i];
            
            // 【死亡禁区判断】：
            // X > 0.05 且 X < 0.3 米 (正前方 5厘米到30厘米内)
            // std::abs(y) < 0.2 米 (左右宽度在 40厘米 的车身范围内)
            if (x > 0.05f && x < 0.30f && std::abs(y) < 0.20f) {
                danger_detected = true;
                break; // 只要发现一个点在这个区域，立刻终止循环，准备急刹！
            }
        }

        // 3. 执行脑干级急刹车！
        if (danger_detected) {
            geometry_msgs::msg::Twist stop_msg;
            stop_msg.linear.x = 0.0;
            stop_msg.angular.z = 0.0;
            cmd_vel_pub_->publish(stop_msg); // 强制覆盖底盘速度！
            
            // 打印红色警报日志！
            RCLCPP_WARN(this->get_logger(), "🚨 触发底层脑干反射！前方0.3米内发现障碍物，已在 %.2f 微秒内强制下发刹车指令！", us_rvv.count());
        }

        // 性能监控：每隔100帧安静地打印一次耗时，证明咱们一直在工作
        static int frame_count = 0;
        if (++frame_count % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "⚡ [性能监控] 安全护卫运行中。单帧 %zu 点坐标解算耗时: %.2f 微秒", n, us_rvv.count());
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RvvLidarProcessor>());
    rclcpp::shutdown();
    return 0;
}