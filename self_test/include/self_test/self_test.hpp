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

#ifndef SELF_TEST__SELF_TEST_HPP_
#define SELF_TEST__SELF_TEST_HHP_

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/srv/self_test.hpp"

#include "diagnostic_updater/diagnostic_updater.hpp"

#include "rcutils/cmdline_parser.h"

#include "rclcpp/rclcpp.hpp"

namespace self_test
{

class TestRunner : public diagnostic_updater::DiagnosticTaskVector
{
private:
  rclcpp::Service<diagnostic_msgs::srv::SelfTest>::SharedPtr service_server_;
  rclcpp::Node::SharedPtr private_node_handle_;
  std::string id_;
  bool verbose;

public:
  using diagnostic_updater::DiagnosticTaskVector::add;

  explicit TestRunner(rclcpp::Node::SharedPtr ph)
  {
    private_node_handle_ = ph;
    auto logger = private_node_handle_->get_logger();
    RCLCPP_DEBUG(logger, "Advertising self_test");

    auto serviceCB =
      [this, &logger](std::shared_ptr<diagnostic_msgs::srv::SelfTest::Request> request,
        std::shared_ptr<diagnostic_msgs::srv::SelfTest::Response> response) -> bool
      {
        bool retval = false;
        bool ignore_set_id_warn = false;

        if (rclcpp::ok()) {
          const std::string unspecified_id("unspecified");

          RCLCPP_INFO(logger, "Entering self-test.");

          std::vector<diagnostic_msgs::msg::DiagnosticStatus> status_vec;

          const std::vector<DiagnosticTaskInternal> & tasks = getTasks();
          for (std::vector<DiagnosticTaskInternal>::const_iterator iter = tasks.begin();
            iter != tasks.end(); iter++)
          {
            diagnostic_updater::DiagnosticStatusWrapper status;

            status.level = 2;
            status.message = "No message was set";

            try {
              RCLCPP_INFO(logger, "Starting test: %s", iter->getName().c_str());
              iter->run(status);
            } catch (std::exception & e) {
              status.level = 2;
              status.message = std::string("Uncaught exception: ") + e.what();
              ignore_set_id_warn = true;
            }

            if (status.level >= 1) {
              if (verbose) {
                RCLCPP_WARN(logger,
                  "Non-zero self-test test status. Name: %s Status %i: Message: %s",
                  status.name.c_str(), status.level, status.message.c_str());
              }
            }
            status_vec.push_back(status);
          }
          if (!ignore_set_id_warn && id_.empty()) {
            RCLCPP_WARN(logger, "setID was not called by any self-test.");
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
            RCLCPP_WARN(logger,
              "Self-test passed, but setID was not called. This is a bug in the driver.");
          }
          response->status = status_vec;

          retval = true;

          RCLCPP_INFO("Self-test complete.");
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
