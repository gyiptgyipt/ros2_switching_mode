#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <string>
#include <memory>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>

class SwitchModeNode : public rclcpp::Node {
public:
    SwitchModeNode()
        : Node("switch_mode") {
        // Subscriber to listen to mode changes
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "robot_api/robot_mode", 10,
            std::bind(&SwitchModeNode::modeCallback, this, std::placeholders::_1));

        // Initial launch setup (slam_toolbox)
        current_mode_ = "slam_toolbox";
        startLaunch(slam_toolbox_package_, slam_toolbox_launch_);
    }

private:
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> subscription_;
    std::string current_mode_;

    // Package and launch file names
    const std::string slam_toolbox_package_ = "slam_toolbox";
    const std::string slam_toolbox_launch_ = "online_async_launch.py";

    const std::string navigation_package_ = "nav2_bringup";
    const std::string navigation_launch_ = "navigation_launch.py";

    const std::string gazebo_package_ = "turtlebot3_gazebo";
    const std::string gazebo_launch_ = "turtlebot3_world.launch.py";

    // Process handle for the launch
    pid_t launch_pid_ = -1;

    void modeCallback(const std_msgs::msg::String::SharedPtr msg) {
        const std::string &mode = msg->data;

        if (mode == current_mode_) {
            RCLCPP_INFO(this->get_logger(), "Mode '%s' is already active.", mode.c_str());
            return;
        }

        // Shutdown current launch
        shutdownLaunch();

        // Start the appropriate launch file
        if (mode == "slam_toolbox") {
            startLaunch(slam_toolbox_package_, slam_toolbox_launch_);
        } else if (mode == "navigation") {
            startLaunch(navigation_package_, navigation_launch_);
        } else if (mode == "gazebo") {
            startLaunch(gazebo_package_, gazebo_launch_);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown mode '%s'.", mode.c_str());
            return;
        }

        current_mode_ = mode;
        RCLCPP_INFO(this->get_logger(), "Switched to mode '%s'.", mode.c_str());
    }

    void startLaunch(const std::string &package, const std::string &launch_file) {
        RCLCPP_INFO(this->get_logger(), "Starting launch file: %s/%s", package.c_str(), launch_file.c_str());

        // Fork a process to run the launch file
        launch_pid_ = fork();
        if (launch_pid_ == 0) {
            // In child process
            execlp("ros2", "ros2", "launch", package.c_str(), launch_file.c_str(), (char *)NULL);
            perror("execlp failed");
            std::exit(EXIT_FAILURE);
        }

        if (launch_pid_ < 0) {
            perror("fork failed");
            throw std::runtime_error("Failed to start launch process");
        }
    }

    void shutdownLaunch() {
        if (launch_pid_ > 0) {
            RCLCPP_INFO(this->get_logger(), "Shutting down current launch process (PID: %d)...", launch_pid_);
            kill(launch_pid_, SIGINT);
            int status;
            waitpid(launch_pid_, &status, 0);
            launch_pid_ = -1;
            sleep(3); // Ensure the process has fully terminated
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwitchModeNode>());
    rclcpp::shutdown();
    return 0;
}
