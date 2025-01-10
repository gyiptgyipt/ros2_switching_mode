
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <vector>
#include <cstdlib>

class NodeSwitcher : public rclcpp::Node
{
public:
    NodeSwitcher()
        : Node("node_switcher")
    {
        // Declare the parameter for the active node
        this->declare_parameter<std::string>("active_node", "none");

        // Add a callback for parameter updates
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&NodeSwitcher::on_parameter_change, this, std::placeholders::_1));

        // Initialize process handles
        mapping_process_ = nullptr;
        localization_process_ = nullptr;
    }

private:
    rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters)
        {
            if (param.get_name() == "active_node")
            {
                std::string active_node = param.as_string();
                RCLCPP_INFO(this->get_logger(), "Switching to node: %s", active_node.c_str());

                if (active_node == "mapping")
                {
                    stop_node(localization_process_);
                    start_node(mapping_process_, "ros2 run my_package mapping_node");
                }
                else if (active_node == "localization")
                {
                    stop_node(mapping_process_);
                    start_node(localization_process_, "ros2 run my_package localization_node");
                }
                else if (active_node == "none")
                {
                    stop_node(mapping_process_);
                    stop_node(localization_process_);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Invalid node name: %s", active_node.c_str());
                    result.successful = false;
                }
            }
        }

        return result;
    }

    void start_node(std::unique_ptr<std::thread> &process_handle, const std::string &command)
    {
        if (process_handle && process_handle->joinable())
        {
            RCLCPP_WARN(this->get_logger(), "Node is already running.");
            return;
        }

        process_handle = std::make_unique<std::thread>([command]() {
            std::system(command.c_str());
        });

        RCLCPP_INFO(this->get_logger(), "Started node with command: %s", command.c_str());
    }

    void stop_node(std::unique_ptr<std::thread> &process_handle)
    {
        if (process_handle && process_handle->joinable())
        {
            process_handle->detach(); // Detach before ending to avoid hanging
            process_handle.reset();
            RCLCPP_INFO(this->get_logger(), "Stopped node.");
        }
    }

    std::unique_ptr<std::thread> mapping_process_;
    std::unique_ptr<std::thread> localization_process_;
    rclcpp::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeSwitcher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}