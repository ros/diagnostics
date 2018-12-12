// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SELF_TEST__SELF_TEST_HPP_
#define SELF_TEST__SELF_TEST_HPP_

#include <stdexcept>
#include <vector>
#include <string>
#include <functional>
#include <thread>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/srv/self_test.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"

namespace self_test
{
//  using namespace diagnostic_updater;

class TestRunner : public diagnostic_updater::DiagnosticTaskVector
{
private:
  rclcpp::Service<diagnostic_msgs::srv::SelfTest>::SharedPtr service_server_;
  rclcpp::Node::SharedPtr node_handle_;
  rclcpp::Node::SharedPtr private_node_handle_;
  std::string id_;
  bool verbose;

public:
  using diagnostic_updater::DiagnosticTaskVector::add;

  explicit TestRunner(rclcpp::Node::SharedPtr ph)
  {
    //  ROS_DEBUG("Advertising self_test");
    private_node_handle_ = ph;

    auto serviceCB =
      [this](std::shared_ptr<diagnostic_msgs::srv::SelfTest::Request> request,
        std::shared_ptr<diagnostic_msgs::srv::SelfTest::Response> response) -> bool
      {
        bool retval = false;
        bool ignore_set_id_warn = false;
        std::cout << "I am in service callback" << std::endl;

        if (rclcpp::ok()) {
          const std::string unspecified_id("unspecified");

          //  ROS_INFO("Entering self-test.");
          std::cout << "Entering self-test." << std::endl;

          std::vector<diagnostic_msgs::msg::DiagnosticStatus> status_vec;

          const std::vector<DiagnosticTaskInternal> & tasks = getTasks();
          for (std::vector<DiagnosticTaskInternal>::const_iterator iter = tasks.begin();
            iter != tasks.end(); iter++)
          {
            diagnostic_updater::DiagnosticStatusWrapper status;

            status.level = 2;
            status.message = "No message was set";

            try {
              //  ROS_INFO("Starting test: %s", iter->getName().c_str());
              std::cout << "Starting test: %s" << iter->getName().c_str() << std::endl;
              iter->run(status);
            } catch (std::exception & e) {
              status.level = 2;
              status.message = std::string("Uncaught exception: ") + e.what();
              ignore_set_id_warn = true;
            }

            if (status.level >= 1) {
              if (verbose) {
                std::cout << "Non-zero self-test test status. Name:" << status.name.c_str() <<
                  "status %i: '%s" << status.level << "msg is" << status.message << std::endl;
              }
            }
            status_vec.push_back(status);
          }
          if (!ignore_set_id_warn && id_.empty()) {
            std::cout << "setID was not called by any self-test" << std::endl;
          }
          //  One of the test calls should use setID
          response->id = id_;

          response->passed = true;
          for (std::vector<diagnostic_msgs::msg::DiagnosticStatus>::iterator status_iter =
            status_vec.begin();
            status_iter != status_vec.end();
            status_iter++)
          {
            if (status_iter->level >= 2) {
              response->passed = false;
            }
          }


          if (response->passed && id_ == unspecified_id) {
            std::cout <<
              "Self-test passed, but setID was not called. This is a bug in the driver."
                      <<
              std::endl;
          }
          response->status = status_vec;

          retval = true;

          std::cout << "Self-test complete." << std::endl;
        }

        return retval;
      };

    service_server_ = private_node_handle_->create_service<diagnostic_msgs::srv::SelfTest>(
      "self_test", serviceCB);
    verbose = true;
  }
  void setID(std::string id)
  {
    id_ = id;
  }
};
}  //  namespace self_test
#endif  //  SELF_TEST__SELF_TEST_HPP_
