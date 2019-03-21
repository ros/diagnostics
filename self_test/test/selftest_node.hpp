/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SELFTEST_NODE_HPP_
#define SELFTEST_NODE_HPP_

#include <stdexcept>
#include <string>

#include "self_test/test_runner.hpp"

/*
 *\author Kevin Watts
 *\brief Returns nominal self-test values
 */
class SelftestNode : public rclcpp::Node
{
protected:
  self_test::TestRunner self_test_;
  double some_val_;

public:
  explicit SelftestNode(const std::string & node_name)
  : rclcpp::Node(node_name),
    self_test_(
      get_node_base_interface(), get_node_services_interface(), get_node_logging_interface())
  {
    setup_test_cases();
  }

  virtual ~SelftestNode() = default;

  virtual void setup_test_cases()
  {
    self_test_.add("Pretest", this, &SelftestNode::pretest);

    self_test_.add("ID Lookup", this, &SelftestNode::test1);
    self_test_.add("Exception generating test", this, &SelftestNode::test2);
    self_test_.add("Value generating test", this, &SelftestNode::test3);
    self_test_.add("Value testing test", this, &SelftestNode::test4);

    self_test_.add("Posttest", this, &SelftestNode::pretest);
  }

  virtual void pretest(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    RCLCPP_INFO(this->get_logger(), "Doing preparation stuff before we run our test.");
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Pretest completed successfully.");
    some_val_ = 1.0;
  }

  virtual void test1(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    // Look up ID here
    char ID[] = "12345";
    bool lookup_successful = true;

    if (lookup_successful) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "ID Lookup successful");
      self_test_.setID(ID);
    } else {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "ID Lookup failed");
    }
  }

  // Tests do not necessarily need to catch their exceptions.
  virtual void test2(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    status.level = 0;

    // Here's where we would report success if we'd made it past
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "We made it past the exception throwing statement.");
  }

  virtual void test3(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    some_val_ += 41.0;

    status.add("some value", some_val_);
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK, "We successfully changed the value.");
  }

  virtual void test4(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    if (some_val_ == 42.0) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "We observed the change in value");
    } else {
      status.summaryf(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "We failed to observe the change in value, it is currently %f.", some_val_);
    }
  }

  virtual void posttest(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    RCLCPP_INFO(this->get_logger(), "Doing cleanup stuff after we run our test.");
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Posttest completed successfully.");
  }
};
#endif  // SELFTEST_NODE_HPP_
