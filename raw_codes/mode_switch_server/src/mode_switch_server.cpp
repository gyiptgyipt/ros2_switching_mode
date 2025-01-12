#include "rclcpp/rclcpp.hpp"
#include "mode_switch_interfaces/srv/mode_switch.hpp"  // Include the service message
#include <thread>
#include <memory>
#include <vector>
#include <cstdlib>
#include <signal.h>  // For kill()

using namespace std::placeholders;

class ModeSwitchServer : public rclcpp::Node {
public:
    ModeSwitchServer() : Node("mode_switch_server") {
        this->declare_parameter("current_mode", mode_switch_interfaces::srv::ModeSwitch::Request::MODE_NAVIGATION);
        current_mode_ = this->get_parameter("current_mode").as_int();

        service_ = this->create_service<mode_switch_interfaces::srv::ModeSwitch>(
            "mode_switch",
            std::bind(&ModeSwitchServer::handle_mode_switch, this, _1, _2)
        );

        RCLCPP_INFO(this->get_logger(), "Mode Switch Server started. Current mode: %d", current_mode_);
    }

private:
    void handle_mode_switch(
        const std::shared_ptr<mode_switch_interfaces::srv::ModeSwitch::Request> request,
        std::shared_ptr<mode_switch_interfaces::srv::ModeSwitch::Response> response) {

        RCLCPP_INFO(this->get_logger(), "Received mode switch request: %d", request->mode);

        if (request->mode != mode_switch_interfaces::srv::ModeSwitch::Request::MODE_NAVIGATION &&
            request->mode != mode_switch_interfaces::srv::ModeSwitch::Request::MODE_MAPPING &&
            request->mode != mode_switch_interfaces::srv::ModeSwitch::Request::MODE_REMAPPING) {
            response->success = false;
            RCLCPP_ERROR(this->get_logger(), "Invalid mode requested: %d", request->mode);
            return;
        }

        stop_current_processes();

        if (start_processes_for_mode(request->mode)) {
            current_mode_ = request->mode;
            RCLCPP_INFO(this->get_logger(), "Switched to mode: %d", current_mode_);
            response->success = true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to switch to mode: %d", request->mode);
            response->success = false;
        }
    }

    void stop_current_processes() {
        if (!process_pids_.empty()) {
            RCLCPP_INFO(this->get_logger(), "Stopping current processes...");
            for (pid_t pid : process_pids_) {
                if (kill(pid, SIGTERM) == 0) {  // Send SIGTERM to gracefully terminate the process
                    RCLCPP_INFO(this->get_logger(), "Terminated process with PID: %d", pid);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to terminate process with PID: %d", pid);
                }
            }
            process_pids_.clear();
        }
    }

    bool start_processes_for_mode(uint8_t mode) {
        std::string launch_file;
        switch (mode) {
            case mode_switch_interfaces::srv::ModeSwitch::Request::MODE_NAVIGATION:
                launch_file = "navigation_launch.py";
                break;
            case mode_switch_interfaces::srv::ModeSwitch::Request::MODE_MAPPING:
                launch_file = "mapping_launch.py";
                break;
            case mode_switch_interfaces::srv::ModeSwitch::Request::MODE_REMAPPING:
                launch_file = "remapping_launch.py";
                break;
            default:
                return false;
        }

        try {
            pid_t pid = fork();  // Create a child process
            if (pid == 0) {
                // Child process
                execlp("ros2", "ros2", "launch", "mode_switch_server", launch_file.c_str(), (char *)nullptr);
                std::exit(EXIT_FAILURE);  // Exit if execlp fails
            } else if (pid > 0) {
                // Parent process
                process_pids_.push_back(pid);  // Store the child process PID
                RCLCPP_INFO(this->get_logger(), "Started process with PID: %d for mode: %d", pid, mode);
                return true;
            } else {
                // Fork failed
                RCLCPP_ERROR(this->get_logger(), "Failed to fork process for mode: %d", mode);
                return false;
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start process: %s", e.what());
            return false;
        }
    }

    int current_mode_;
    rclcpp::Service<mode_switch_interfaces::srv::ModeSwitch>::SharedPtr service_;
    std::vector<pid_t> process_pids_;  // Store PIDs of launched processes
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto server = std::make_shared<ModeSwitchServer>();
    rclcpp::spin(server);
    rclcpp::shutdown();
    return 0;
}
