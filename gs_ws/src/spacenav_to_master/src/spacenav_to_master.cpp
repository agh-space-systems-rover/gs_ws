#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "kalman_interfaces/msg/master_message.hpp"

class TwistRepublisher : public rclcpp::Node
{
public:
    TwistRepublisher() : Node("spacenav_to_master")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/spacenav/twist", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                republishMasterMessage(msg);
            });

        publisher_ = this->create_publisher<kalman_interfaces::msg::MasterMessage>(
            "/master_com/ros_to_master", 10);
    }

private:
    void republishMasterMessage(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto master_msg = std::make_shared<kalman_interfaces::msg::MasterMessage>();
        master_msg->cmd = kalman_interfaces::msg::MasterMessage().ARM_SEND_SPACEMOUSE;
        master_msg->data.resize(6);
        master_msg->data[0] = convert_twist_data(msg->linear.x);
        master_msg->data[1] = convert_twist_data(msg->linear.y);
        master_msg->data[2] = convert_twist_data(msg->linear.z);
        master_msg->data[3] = convert_twist_data(msg->angular.x);
        master_msg->data[4] = convert_twist_data(msg->angular.y);
        master_msg->data[5] = convert_twist_data(msg->angular.z);
        publisher_->publish(*master_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<kalman_interfaces::msg::MasterMessage>::SharedPtr publisher_;

    uint8_t convert_twist_data(double twist_data) const {
      return uint8_t((twist_data + 1.0) / 2.0 * 255 + 0.5);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistRepublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}