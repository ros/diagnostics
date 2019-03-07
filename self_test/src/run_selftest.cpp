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
  ClientNode()
  : Node("self_test_client")
  {
    client_ = create_client<diagnostic_msgs::srv::SelfTest>("self_test");

    //  Queue an asynchronous service request that will be sent once `spin` is called on the node.
    queue_async_request();
  }

  void queue_async_request()
  {
    using namespace std::chrono_literals;
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    auto request = std::make_shared<diagnostic_msgs::srv::SelfTest::Request>();

    // We give the async_send_request() method a callback that will get executed once the response
    // is received.
    // This way we can return immediately from this method and allow other work to be done by the
    // executor in `spin` while waiting for the response.
    using ServiceResponseFuture =
      rclcpp::Client<diagnostic_msgs::srv::SelfTest>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result_out = future.get();
        std::string passfail;

        if (result_out->passed) {
          passfail = "PASSED";
        } else {
          passfail = "FAILED";
        }

        printf("Self test %s for device with id: [%s]\n", passfail.c_str(), result_out->id.c_str());

        for (size_t i = 0; i < result_out->status.size(); i++) {
          printf("%2zd) %s\n", i + 1, result_out->status[i].name.c_str());
          if (result_out->status[i].level == 0) {
            printf("     [OK]: ");
          } else if (result_out->status[i].level == 1) {
            printf("     [WARNING]: ");
          } else {
            printf("     [ERROR]: ");
          }
          printf("%s\n", result_out->status[i].message.c_str());

          for (size_t j = 0; j < result_out->status[i].values.size(); j++) {
            printf("      [%s] %s\n",
              result_out->status[i].values[j].key.c_str(),
              result_out->status[i].values[j].value.c_str());
          }
          printf("\n");
        }

        rclcpp::shutdown();
      };
    auto future_result = client_->async_send_request(request, response_received_callback);
  }

private:
  rclcpp::Client<diagnostic_msgs::srv::SelfTest>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ClientNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
