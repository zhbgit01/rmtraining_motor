#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <memory>

class MotorSimulator : public rclcpp::Node
{
public:
    MotorSimulator() : Node("motor_simulator")
    {
        // 电机参数
        J = 1.0;      // 转动惯量
        B = 0.1;      // 粘性阻尼
        TL = 0.0;     // 负载力矩

        // 初始状态
        theta = 0.0;  // 角度
        omega = 0.0;  // 角速度
        torque = 0.0; // 控制力矩

        // 接收控制力矩
        torque_subscriber = this->create_subscription<std_msgs::msg::Float64>(
            "motor_torque",
            10,
            [this](const std_msgs::msg::Float64::SharedPtr msg)
            {
                torque = msg->data;
            });

        // 发布角速度
        speed_publisher = this->create_publisher<std_msgs::msg::Float64>(
            "motor_speed", 10);

        // 发布角度
        angle_publisher = this->create_publisher<std_msgs::msg::Float64>(
            "motor_angle", 10);

        // 1ms更新一次，也就是1000Hz
        timer = this->create_wall_timer(
            std::chrono::milliseconds(1),
            [this]()
            {
                update_motor();
            });

        RCLCPP_INFO(this->get_logger(),
                    "Motor simulator started. Update frequency: 1000 Hz");
    }

private:
    void update_motor()
    {
        // 时间步长
        double dt = 0.001;

        // 电机动力学：
        // J * omega_dot = Te - TL - B * omega

        double omega_dot =
            (torque - TL - B * omega) / J;

        // 更新角速度
        omega += omega_dot * dt;

        // 更新角度
        theta += omega * dt;

        // 发布角速度
        std_msgs::msg::Float64 speed_msg;
        speed_msg.data = omega;
        speed_publisher->publish(speed_msg);

        // 发布角度
        std_msgs::msg::Float64 angle_msg;
        angle_msg.data = theta;
        angle_publisher->publish(angle_msg);
    }

    // 电机参数
    double J;
    double B;
    double TL;

    // 电机状态
    double theta;
    double omega;

    // 当前控制力矩
    double torque;

    // ROS2
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr torque_subscriber;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_publisher;

    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MotorSimulator>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}