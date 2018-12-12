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

#include <stdexcept>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/srv/self_test.hpp"
#include "self_test/self_test.hpp"


class MyNode
{
private:
  self_test::TestRunner self_test_;
  double some_val;

public:
  explicit MyNode(rclcpp::Node::SharedPtr nh_)
  : self_test_(nh_)
  {
    self_test_.add("Pretest", this, &MyNode::pretest);

    self_test_.add("ID Lookup", this, &MyNode::test1);
    self_test_.add("Error test", this, &MyNode::test2);
    self_test_.add("Value generating test", this, &MyNode::test3);
    self_test_.add("Value testing test", this, &MyNode::test4);

    self_test_.add("Posttest", this, &MyNode::pretest);
  }

  void pretest(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    //  ROS_INFO("Doing preparation stuff before we run our test.\n");
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Pretest completed successfully.");

    some_val = 1.0;
  }

  void test1(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    //  Look up ID here
    char ID[] = "12345";
    bool lookup_successful = true;

    if (lookup_successful) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "ID Lookup successful");
      self_test_.setID(ID);
    } else {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "ID Lookup failed");
    }
  }

  void test2(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    //  Report an error, this should fail self test
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Reporting an error");
  }

  void test3(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    some_val += 41.0;
    status.add("some value", some_val);
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
      "We successfully changed the value.");
  }

  void test4(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    if (some_val == 42.0) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "We observed the change in value");
    } else {
      status.summaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "We failed to observe the change in value, it is currently %f.", some_val);
    }
  }

  void posttest(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    //  ROS_INFO("Doing cleanup stuff after we run our test.\n");
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Posttest completed successfully.");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nh_;
  nh_ = std::make_shared<rclcpp::Node>("my_node");
  MyNode n(nh_);
  rclcpp::spin(nh_);
  return 0;
}
