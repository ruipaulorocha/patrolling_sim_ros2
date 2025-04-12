#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

class TwistStampedToTwistNode : public rclcpp::Node
{
public:
    TwistStampedToTwistNode() : Node("twist_stamped_to_twist")
    {
        // Subscriber to /cmd_vel (TwistStamped)
        subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "cmd_vel", 10,
            std::bind(&TwistStampedToTwistNode::twist_callback, this, std::placeholders::_1));
        
        // Publisher for /cmd_vel_not_stamped (Twist)
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_not_stamped", 10);
    }

private:
    void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        // Create a Twist message and copy the velocity data
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear = msg->twist.linear;
        twist_msg.angular = msg->twist.angular;
        
        // Publish the new message
        publisher_->publish(twist_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistStampedToTwistNode>());
    rclcpp::shutdown();
    return 0;
}
