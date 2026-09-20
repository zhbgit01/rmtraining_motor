#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <cmath>
#include <memory>

class InputSource : public rclcpp::Node
{
public:
    InputSource() : Node("input_source")
    {
        publisher = this->create_publisher<std_msgs::msg::Float64>(
            "motor_torque", 10);

        // 100Hz发送控制指令
        timer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            [this]()
            {
                publish_torque();
            });

        start_time = this->now();

        RCLCPP_INFO(this->get_logger(),
                    "Input source started. Control frequency: 100 Hz");
    }

private:
    void publish_torque()
    {
        double t = (this->now() - start_time).seconds();

        double torque = 0.0;

        // 0~2秒：+1 Nm
        if (t < 2.0)
        {
            torque = 1.0;
        }
        // 2~4秒：0 Nm
        else if (t < 4.0)
        {
            torque = 0.0;
        }
        // 4~6秒：-1 Nm
        else if (t < 6.0)
        {
            torque = -1.0;
        }
        // 6秒以后：0 Nm
        else
        {
            torque = 0.0;
        }

        std_msgs::msg::Float64 msg;
        msg.data = torque;

        publisher->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher;

    rclcpp::TimerBase::SharedPtr timer;

    rclcpp::Time start_time;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<InputSource>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}