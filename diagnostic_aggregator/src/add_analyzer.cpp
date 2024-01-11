#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

class AddAnalyzer : public rclcpp::Node
{
public:
  AddAnalyzer()
  : Node("add_analyzer_node", "", rclcpp::NodeOptions().allow_undeclared_parameters(
        true).automatically_declare_parameters_from_overrides(true))
  {
    client_ = this->create_client<rcl_interfaces::srv::SetParametersAtomically>(
      "/diagnostics_agg/set_parameters_atomically");
  }

  void send_request()
  {
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO_ONCE(this->get_logger(), "service not available, waiting ...");
    }
    auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
    std::map<std::string, rclcpp::Parameter> parameters;
    if (this->get_parameters("", parameters)) {
      for (const auto & param : parameters) {
        if (param.first.substr(0, prefix_.length()).compare(prefix_) == 0) {
          auto parameter_msg = param.second.to_parameter_msg();
          request->parameters.push_back(parameter_msg);
        }
      }
    }
    auto result = client_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Parameters succesfully set");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to set parameters");
    }
  }

private:
  rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr client_;
  std::string prefix_ = "analyzers.";
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto add_analyzer = std::make_shared<AddAnalyzer>();
  add_analyzer->send_request();
  rclcpp::shutdown();

  return 0;
}
