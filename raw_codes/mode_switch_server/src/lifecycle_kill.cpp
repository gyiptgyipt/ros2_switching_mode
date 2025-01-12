#include <rclcpp/rclcpp.hpp>
#include <cstdlib> // For std::system
#include <thread>  // For std::thread
#include <chrono>  // For std::chrono

class LaunchNode : public rclcpp::Node
{
public:
    LaunchNode() : Node("launch_node")
    {
        // Construct the ros2 run command
        std::string command = "ros2 run turtlesim turtlesim_node";

        // Log the command
        RCLCPP_INFO(this->get_logger(), "Executing command: %s", command.c_str());

        // Execute the command in a separate thread
        std::thread launch_thread([this, command]() {
            int result = std::system(command.c_str());
            if (result == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Command executed successfully.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to execute command.");
            }
        });

        // Detach the thread to run independently
        launch_thread.detach();

        // Create a timer to kill the process after 10 seconds
        timer_ = this->create_wall_timer(
            std::chrono::seconds(10), // 10 seconds
            [this]() {
                RCLCPP_INFO(this->get_logger(), "Killing the launched process...");
                std::system("pkill -f turtlesim_node"); // Kill the process
                rclcpp::shutdown(); // Shutdown the node
            }
        );
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaunchNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}