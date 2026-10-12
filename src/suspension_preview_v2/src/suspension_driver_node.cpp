// ============================================================================
//  suspension_driver_node.cpp  —  悬架驱动节点
//
//  把 ISuspensionDriver 接口通过 ROS 话题暴露出来，方便联调/调用：
//     /suspension/mode            std_msgs/Int32          0=被动 1=找平 2=平衡 3=紧急
//     /suspension/target_height   std_msgs/Int32MultiArray 四轮目标高度 (mm)
//                                 [左前, 右前, 左后, 右后]
//     /suspension/emergency_stop  std_msgs/Bool           true=急停
//
//  用法示例：
//     ros2 topic pub --once /suspension/mode std_msgs/msg/Int32 "{data: 2}"
//     ros2 topic pub --once /suspension/target_height \
//         std_msgs/msg/Int32MultiArray "{data: [10, -10, 5, -5]}"
//     ros2 topic pub --once /suspension/emergency_stop std_msgs/msg/Bool "{data: true}"
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <array>
#include <memory>
#include <string>

#include "suspension_preview_v2/suspension_driver.hpp"

using suspension_preview_v2::ISuspensionDriver;
using suspension_preview_v2::SuspensionMode;

class SuspensionDriverNode : public rclcpp::Node {
public:
    SuspensionDriverNode() : rclcpp::Node("suspension_driver_node") {
        const auto device = declare_parameter<std::string>("serial_port", "/dev/ttyS0");
        const auto baud = declare_parameter<int>("baud_rate", 115200);

        driver_ = suspension_preview_v2::make_serial_driver(device, baud);
        if (driver_->connect()) {
            RCLCPP_INFO(get_logger(), "悬架驱动已连接: %s @ %d bps", device.c_str(), baud);
        } else {
            RCLCPP_ERROR(get_logger(), "悬架驱动连接失败: %s", driver_->last_error().c_str());
        }

        sub_mode_ = create_subscription<std_msgs::msg::Int32>(
            "/suspension/mode", 10, [this](std_msgs::msg::Int32::SharedPtr m) {
                if (!is_valid_mode(m->data)) {
                    RCLCPP_WARN(get_logger(), "非法模式: %d（应为 0-3）", m->data);
                    return;
                }
                const auto mode = static_cast<SuspensionMode>(m->data);
                if (driver_->set_mode(mode)) {
                    RCLCPP_INFO(get_logger(), "模式已切换: %d", m->data);
                } else {
                    RCLCPP_ERROR(get_logger(), "设置模式失败: %s",
                                 driver_->last_error().c_str());
                }
            });

        sub_height_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            "/suspension/target_height", 10,
            [this](std_msgs::msg::Int32MultiArray::SharedPtr m) {
                if (m->data.size() != static_cast<std::size_t>(
                                           suspension_preview_v2::kSuspensionWheelCount)) {
                    RCLCPP_WARN(get_logger(), "需要 4 个高度值 [左前,右前,左后,右后]，收到 %zu 个",
                                m->data.size());
                    return;
                }
                std::array<int, suspension_preview_v2::kSuspensionWheelCount> h{};
                for (std::size_t i = 0; i < h.size(); ++i) h[i] = m->data[i];
                if (driver_->set_target_height(h)) {
                    RCLCPP_INFO(get_logger(), "目标高度已下发: [%d, %d, %d, %d] mm",
                                h[0], h[1], h[2], h[3]);
                } else {
                    RCLCPP_ERROR(get_logger(), "下发目标高度失败: %s",
                                 driver_->last_error().c_str());
                }
            });

        sub_estop_ = create_subscription<std_msgs::msg::Bool>(
            "/suspension/emergency_stop", 10, [this](std_msgs::msg::Bool::SharedPtr m) {
                if (!m->data) return;
                if (driver_->emergency_stop()) {
                    RCLCPP_WARN(get_logger(), "已触发急停");
                } else {
                    RCLCPP_ERROR(get_logger(), "急停指令发送失败: %s",
                                 driver_->last_error().c_str());
                }
            });

        RCLCPP_INFO(get_logger(),
                    "驱动节点就绪 | 话题: /suspension/mode, /suspension/target_height, "
                    "/suspension/emergency_stop");
    }

    ~SuspensionDriverNode() override {
        if (driver_) driver_->disconnect();
    }

private:
    static bool is_valid_mode(int v) {
        return v >= static_cast<int>(SuspensionMode::Passive) &&
               v <= static_cast<int>(SuspensionMode::Emergency);
    }

    std::unique_ptr<ISuspensionDriver> driver_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_mode_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_height_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SuspensionDriverNode>());
    rclcpp::shutdown();
    return 0;
}
