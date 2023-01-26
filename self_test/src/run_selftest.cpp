/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include <chrono>
#include <memory>
#include <string>

#include "diagnostic_msgs/srv/self_test.hpp"
#include "rclcpp/rclcpp.hpp"

class ClientNode : public rclcpp::Node
{
public:
  explicit ClientNode(std::string _node_name)
  : Node("self_test_client"), node_name_to_test(_node_name)
  {
    client_ = create_client<diagnostic_msgs::srv::SelfTest>(
      node_name_to_test +
      "/self_test");
  }

  rclcpp::Client<diagnostic_msgs::srv::SelfTest>::SharedFuture
  queue_async_request()
  {
    using namespace std::chrono_literals;
    using ServiceResponseFuture =
      rclcpp::Client<diagnostic_msgs::srv::SelfTest>::SharedFuture;

    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Interrupted while waiting for the service. Exiting.");
        return ServiceResponseFuture();
      }
      RCLCPP_INFO(
        this->get_logger(),
        "service not available, waiting again...");
    }
    auto request = std::make_shared<diagnostic_msgs::srv::SelfTest::Request>();

    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result_out = future.get();

        RCLCPP_INFO(
          this->get_logger(), "Self test %s for device with id: [%s]",
          (result_out->passed ? "PASSED" : "FAILED"),
          result_out->id.c_str());

        // for (size_t i = 0; i < result_out->status.size(); i++) {
        auto counter = 1lu;
        for (const auto & status : result_out->status) {
          RCLCPP_INFO(
            this->get_logger(), "%zu) %s", counter++,
            status.name.c_str());
          if (status.level == 0) {
            RCLCPP_INFO(this->get_logger(), "\t%s", status.message.c_str());
          } else if (status.level == 1) {
            RCLCPP_WARN(this->get_logger(), "\t%s", status.message.c_str());
          } else {
            RCLCPP_ERROR(this->get_logger(), "\t%s", status.message.c_str());
          }

          for (const auto & kv : status.values) {
            RCLCPP_INFO(
              this->get_logger(), "\t[%s] %s", kv.key.c_str(),
              kv.value.c_str());
          }
        }
      };
#if defined(NO_FUTURE)
    return client_->async_send_request(request, response_received_callback);
#else
    return client_->async_send_request(request, response_received_callback).future;
#endif
  }

private:
  rclcpp::Client<diagnostic_msgs::srv::SelfTest>::SharedPtr client_;
  std::string node_name_to_test;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  if (argc != 2) {
    RCLCPP_ERROR(
      rclcpp::get_logger("run_selftest"),
      "usage: run_selftest NODE_NAME");
    return 1;
  }
  std::string node_name = argv[1];

  auto node = std::make_shared<ClientNode>(node_name);
  auto async_srv_request = node->queue_async_request();
  if (!async_srv_request.valid()) {
    rclcpp::shutdown();
    return -1;
  }

  rclcpp::spin_until_future_complete(node, async_srv_request);
  rclcpp::shutdown();
  return 0;
}
