#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <cmath>
#include <memory>

class DualPIDController : public rclcpp::Node
{
public:
    DualPIDController() : Node("dual_pid_controller")
    {
        // =========================
        // 目标位置
        // =========================
        // 这里设置最终目标位置
        // 例如 90°，程序会自动选择优弧：-270°
        target_position = 90.0 * M_PI / 180.0;

        // =========================
        // 外环：位置 PID
        // =========================
        position_kp = 1.5;
        position_ki = 0.0;
        position_kd = 0.0;

        // =========================
        // 内环：速度 PID
        // =========================
        velocity_kp = 4.0;
        velocity_ki = 0.2;
        velocity_kd = 0.05;

        // =========================
        // 状态
        // =========================
        current_angle = 0.0;
        current_speed = 0.0;

        continuous_target = 0.0;
        target_initialized = false;

        position_integral = 0.0;
        previous_position_error = 0.0;

        velocity_integral = 0.0;
        previous_velocity_error = 0.0;

        // =========================
        // 接收电机角度
        // =========================
        angle_subscriber =
            this->create_subscription<std_msgs::msg::Float64>(
                "motor_angle",
                10,
                [this](const std_msgs::msg::Float64::SharedPtr msg)
                {
                    current_angle = msg->data;

                    // 第一次收到角度时，计算优弧目标
                    if (!target_initialized)
                    {
                        calculate_major_arc_target();
                        target_initialized = true;
                    }
                });

        // =========================
        // 接收电机速度
        // =========================
        speed_subscriber =
            this->create_subscription<std_msgs::msg::Float64>(
                "motor_speed",
                10,
                [this](const std_msgs::msg::Float64::SharedPtr msg)
                {
                    current_speed = msg->data;
                });

        // =========================
        // 输出控制力矩
        // =========================
        torque_publisher =
            this->create_publisher<std_msgs::msg::Float64>(
                "motor_torque",
                10);

        // 用于 Foxglove 可视化
        target_angle_publisher =
            this->create_publisher<std_msgs::msg::Float64>(
                "target_angle",
                10);

        target_speed_publisher =
            this->create_publisher<std_msgs::msg::Float64>(
                "target_speed",
                10);

        // =========================
        // 1000 Hz 控制
        // =========================
        timer = this->create_wall_timer(
            std::chrono::milliseconds(1),
            [this]()
            {
                update_controller();
            });

        RCLCPP_INFO(
            this->get_logger(),
            "Dual PID controller started. Control frequency: 1000 Hz");
    }

private:

    // ============================================================
    // 把角度限制到 [-PI, PI)
    // ============================================================
    double normalize_angle(double angle)
    {
        while (angle >= M_PI)
            angle -= 2.0 * M_PI;

        while (angle < -M_PI)
            angle += 2.0 * M_PI;

        return angle;
    }

    // ============================================================
    // 自动计算优弧目标
    // ============================================================
    void calculate_major_arc_target()
    {
        // 当前角度和目标角度都转换到 [-PI, PI)
        double current_normalized =
            normalize_angle(current_angle);

        double target_normalized =
            normalize_angle(target_position);

        // 先计算最短路径
        double short_delta =
            normalize_angle(
                target_normalized - current_normalized);

        // 选择另一条路径，也就是优弧
        double major_delta;

        if (short_delta > 0.0)
        {
            major_delta = short_delta - 2.0 * M_PI;
        }
        else
        {
            major_delta = short_delta + 2.0 * M_PI;
        }

        // 得到连续角度目标
        continuous_target =
            current_angle + major_delta;

        RCLCPP_INFO(
            this->get_logger(),
            "Current angle: %.2f deg",
            current_angle * 180.0 / M_PI);

        RCLCPP_INFO(
            this->get_logger(),
            "Target position: %.2f deg",
            target_position * 180.0 / M_PI);

        RCLCPP_INFO(
            this->get_logger(),
            "Major arc target: %.2f deg",
            continuous_target * 180.0 / M_PI);
    }

    // ============================================================
    // 双环 PID
    // ============================================================
    void update_controller()
    {
        double dt = 0.001;

        if (!target_initialized)
            return;

        // ========================================================
        // 第一层：位置环
        // ========================================================

        double position_error =
            continuous_target - current_angle;

        position_integral +=
            position_error * dt;

        double position_derivative =
            (position_error - previous_position_error) / dt;

        // 位置 PID 输出：目标速度
        double target_speed =
            position_kp * position_error +
            position_ki * position_integral +
            position_kd * position_derivative;

        // 限制最大目标速度
        const double max_speed = 2.0;

        if (target_speed > max_speed)
            target_speed = max_speed;

        if (target_speed < -max_speed)
            target_speed = -max_speed;

        previous_position_error = position_error;

        // ========================================================
        // 第二层：速度环
        // ========================================================

        double velocity_error =
            target_speed - current_speed;

        velocity_integral +=
            velocity_error * dt;

        double velocity_derivative =
            (velocity_error - previous_velocity_error) / dt;

        // 速度 PID 输出：控制力矩
        double torque =
            velocity_kp * velocity_error +
            velocity_ki * velocity_integral +
            velocity_kd * velocity_derivative;

        // 限制最大控制力矩
        const double max_torque = 5.0;

        if (torque > max_torque)
            torque = max_torque;

        if (torque < -max_torque)
            torque = -max_torque;

        previous_velocity_error = velocity_error;

        // ========================================================
        // 发布控制力矩
        // ========================================================

        std_msgs::msg::Float64 torque_msg;
        torque_msg.data = torque;

        torque_publisher->publish(torque_msg);

        // ========================================================
        // 发布目标角度和目标速度
        // 用于 Foxglove 观察
        // ========================================================

        std_msgs::msg::Float64 target_angle_msg;
        target_angle_msg.data = continuous_target;

        target_angle_publisher->publish(
            target_angle_msg);

        std_msgs::msg::Float64 target_speed_msg;
        target_speed_msg.data = target_speed;

        target_speed_publisher->publish(
            target_speed_msg);
    }

    // ============================================================
    // 目标位置
    // ============================================================

    double target_position;

    // 自动计算出的连续优弧目标
    double continuous_target;

    bool target_initialized;

    // 当前电机状态
    double current_angle;
    double current_speed;

    // ============================================================
    // 位置环 PID 参数
    // ============================================================

    double position_kp;
    double position_ki;
    double position_kd;

    double position_integral;
    double previous_position_error;

    // ============================================================
    // 速度环 PID 参数
    // ============================================================

    double velocity_kp;
    double velocity_ki;
    double velocity_kd;

    double velocity_integral;
    double previous_velocity_error;

    // ROS
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        angle_subscriber;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        speed_subscriber;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        torque_publisher;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        target_angle_publisher;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        target_speed_publisher;

    rclcpp::TimerBase::SharedPtr timer;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<DualPIDController>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}