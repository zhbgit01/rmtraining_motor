#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <memory>

class PIDController : public rclcpp::Node
{
public:
    PIDController() : Node("pid_controller")
    {
        // PID 参数
        Kp = 5.0;
        Ki = 0.0;
        Kd = 0.5;

        // 目标角度
        target_angle = 3.14;

        // 当前角度
        current_angle = 0.0;

        // PID 计算频率：1000 Hz
        timer = this->create_wall_timer(
            std::chrono::milliseconds(1),
            [this]()
            {
                update_pid();
            });

        // 接收电机当前角度
        angle_subscriber = this->create_subscription<std_msgs::msg::Float64>(
            "motor_angle",
            10,
            [this](const std_msgs::msg::Float64::SharedPtr msg)
            {
                current_angle = msg->data;
            });

        // 输出控制力矩
        torque_publisher = this->create_publisher<std_msgs::msg::Float64>(
            "motor_torque",
            10);

        previous_error = 0.0;
        integral = 0.0;

        RCLCPP_INFO(this->get_logger(),
                    "PID controller started. Target angle: %.2f rad",
                    target_angle);
    }

private:

    void update_pid()
    {
        // PID计算周期
        double dt = 0.001;

        // 计算误差
        double error = target_angle - current_angle;

        // 积分
        integral += error * dt;

        // 微分
        double derivative = (error - previous_error) / dt;

        // PID输出
        double torque =
            Kp * error +
            Ki * integral +
            Kd * derivative;

        // 限制最大控制力矩
        if (torque > 5.0)
            torque = 5.0;

        if (torque < -5.0)
            torque = -5.0;

        // 发布控制力矩
        std_msgs::msg::Float64 msg;
        msg.data = torque;

        torque_publisher->publish(msg);

        previous_error = error;
    }

    // PID参数
    double Kp;
    double Ki;
    double Kd;

    // 目标角度
    double target_angle;

    // 当前角度
    double current_angle;

    // PID中间变量
    double previous_error;
    double integral;

    rclcpp::TimerBase::SharedPtr timer;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        angle_subscriber;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        torque_publisher;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PIDController>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}