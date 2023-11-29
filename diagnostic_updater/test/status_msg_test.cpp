/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2023, Outrider Technologies, Inc.
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

#include <gtest/gtest.h>

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"


class MockUpdaterNode : public rclcpp::Node
{
public:
  explicit MockUpdaterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mock_updater_node", "test_namespace", options),
    updater_(this)
  {
    updater_.setHardwareID("test_hardware_id");
    updater_.add("test_check", this, &MockUpdaterNode::update_diagnostics);
    updater_.force_update();
  }

  void update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "test message");
  }

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_array_publisher_;
  diagnostic_updater::Updater updater_;
};

class ListenerNode : public rclcpp::Node
{
public:
  ListenerNode()
  : Node("listener_node")
  {
    subscriber_ =
      this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "diagnostics", 10,
      std::bind(&ListenerNode::diagnostics_callback, this, std::placeholders::_1));
  }

  diagnostic_msgs::msg::DiagnosticArray::SharedPtr get_diagnostic_array() const
  {
    return diagnostic_array_;
  }

private:
  void diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    diagnostic_array_ = msg;
  }

  diagnostic_msgs::msg::DiagnosticArray::SharedPtr diagnostic_array_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    subscriber_;
};

TEST(TestStatusMsg, test_name) {
  auto mock_node = std::make_shared<MockUpdaterNode>();
  auto listener_node = std::make_shared<ListenerNode>();

  while (rclcpp::ok() && !listener_node->get_diagnostic_array()) {
    rclcpp::spin_some(mock_node);
    rclcpp::spin_some(listener_node);
  }

  const auto diagnostic_array = listener_node->get_diagnostic_array();
  ASSERT_NE(nullptr, diagnostic_array);
  ASSERT_EQ(1u, diagnostic_array->status.size());
  EXPECT_EQ("mock_updater_node: test_check", diagnostic_array->status[0].name);
  EXPECT_EQ("test_hardware_id", diagnostic_array->status[0].hardware_id);
  EXPECT_EQ("test message", diagnostic_array->status[0].message);
  EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::OK, diagnostic_array->status[0].level);
}

TEST(TestStatusMsg, test_fully_qualified_name) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("diagnostic_updater.use_fqn", true);

  auto mock_node = std::make_shared<MockUpdaterNode>(options);
  auto listener_node = std::make_shared<ListenerNode>();

  while (rclcpp::ok() && !listener_node->get_diagnostic_array()) {
    rclcpp::spin_some(mock_node);
    rclcpp::spin_some(listener_node);
  }

  const auto diagnostic_array = listener_node->get_diagnostic_array();
  ASSERT_NE(nullptr, diagnostic_array);
  ASSERT_EQ(1u, diagnostic_array->status.size());
  EXPECT_EQ("/test_namespace/mock_updater_node: test_check", diagnostic_array->status[0].name);
  EXPECT_EQ("test_hardware_id", diagnostic_array->status[0].hardware_id);
  EXPECT_EQ("test message", diagnostic_array->status[0].message);
  EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::OK, diagnostic_array->status[0].level);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  const int ret = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return ret;
}
