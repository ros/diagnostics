/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Nobleo Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**< \author Martin Cornelis */

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

using namespace std::chrono_literals;

class AddAnalyzer : public rclcpp::Node
{
public:
  AddAnalyzer()
  : Node("add_analyzer_node", "", rclcpp::NodeOptions().allow_undeclared_parameters(
        true).automatically_declare_parameters_from_overrides(true))
  {
    client_ = this->create_client<rcl_interfaces::srv::SetParametersAtomically>(
      "/analyzers/set_parameters_atomically");
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

    if (!this->get_parameters("", parameters)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to retrieve parameters");
    }
    for (const auto & [param_name, param] : parameters) {
      // Find the suffix
      size_t suffix_start = param_name.find_last_of('.');
      // Remove suffix if it exists
      if (suffix_start != std::string::npos) {
        std::string stripped_param_name = param_name.substr(0, suffix_start);
        // Check in map if the stripped param name with the added suffix "path" exists
        // This indicates the parameter is part of an analyzer description
        if (parameters.count(stripped_param_name + ".path") > 0) {
          auto parameter_msg = param.to_parameter_msg();
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
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto add_analyzer = std::make_shared<AddAnalyzer>();
  add_analyzer->send_request();
  rclcpp::shutdown();

  return 0;
}
