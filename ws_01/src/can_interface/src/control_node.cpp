#include <rclcpp/rclcpp.hpp>
#include "pix.h"

using namespace std::chrono_literals;

class PixControlNode : public rclcpp::Node {
public:
    PixControlNode() : Node("pix_control_node") {
        // 初始化控制消息
        drive_msg_ = {
            true,   // 驱动使能
            0,      // 速度控制模式
            1,      // D档
            0.5f,   // 目标速度0.5m/s
            0,      // 油門%
            0, 0    // 自动填充
        };

        steer_msg_ = {
            true,   // 转向使能
            0,      // 转向模式
            100,    // 前轮转向角
            100,    // 后轮转向角
            100,    // 转向速度
            0, 0    // 自动填充
        };

        // 创建定时器保持发送逻辑
        timer_ = this->create_wall_timer(
            2ms, std::bind(&PixControlNode::timer_callback, this));
    }

private:
    void timer_callback() {
        try {
            // 数据赋值
            std::vector<uint8_t> drive_data = encodeDriveControlMessage(drive_msg_);
            std::vector<uint8_t> steer_data = encodeSteerControlMessage(steer_msg_);
            
            // sendCANFrame发送CAN帧
            sendCANFrame("can0", 0x130, drive_data);
            sendCANFrame("can0", 0x132, steer_data);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        }
    }

    // 成员变量
    rclcpp::TimerBase::SharedPtr timer_;
    Message_130 drive_msg_;
    Message_132 steer_msg_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PixControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}