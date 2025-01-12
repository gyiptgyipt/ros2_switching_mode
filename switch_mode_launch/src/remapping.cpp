#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class RemappingPublisher : public rclcpp::Node
{
public:
    RemappingPublisher() : Node("Remapping_Publisher")
    {
        // Create a publisher that publishes String messages to the 'chatter' topic
        publisher_ = this->create_publisher<std_msgs::msg::String>("redirect", 10);

        // Create a timer to publish messages periodically (every 500 milliseconds)
        timer_ = this->create_wall_timer(500ms, std::bind(&RemappingPublisher::publish_message, this));
    }

private:
    void publish_message()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, I am Remapping_mode";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RemappingPublisher>());
    rclcpp::shutdown();
    return 0;
}
