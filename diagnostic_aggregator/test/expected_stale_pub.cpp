// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

using namespace std;
using namespace std::chrono_literals;


class expected_pub : public rclcpp::Node
{
public:
  explicit expected_pub(const std::string & topic_name)
  : Node("dia_pub")
  {
   // msg_ = std::make_shared<std_msgs::msg::String>();
    msg_ = std::make_shared<diagnostic_msgs::msg::DiagnosticArray>();
    msg_s = std::make_shared<diagnostic_msgs::msg::DiagnosticStatus>();
    
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    msg_->header.stamp = clock->now();
    diagnostic_msgs::msg::DiagnosticStatus msg1,msg2,msg3,msg4,msg5,msg6,msg7,msg8,msg9;
    msg1.name="expected1";
    msg1.level=0;
    msg1.message="OK";

    msg2.name="expected2";
    msg2.level=1;
    msg2.message="OK";


    msg3.name="expected3";
    msg3.level=2;
    msg3.message="OK";

    msg4.name="startswith1";
    msg4.level=0;
    msg4.message="OK";

    msg5.name="startswith2";
    msg5.level=0;
    msg5.message="OK";

    msg6.name="startswith3";
    msg6.level=1;
    msg6.message="OK";


    msg8.name="other2";
    msg8.level=0;
    msg8.message="OK";

    msg9.name="other3";
    msg9.level=2;
    msg9.message="OK";
    vector<diagnostic_msgs::msg::DiagnosticStatus> v_msg;
    v_msg.push_back(msg1);
    v_msg.push_back(msg2);
    v_msg.push_back(msg3);
    v_msg.push_back(msg4);
    v_msg.push_back(msg5);
    v_msg.push_back(msg6);
    v_msg.push_back(msg8);
    v_msg.push_back(msg9);

    msg_->status = v_msg;

    auto publish_message =
      [this]() -> void
      {
        RCLCPP_INFO(this->get_logger(), "Publishing:")
        pub_->publish(msg_);
      };
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.depth = 7;
    pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", custom_qos_profile);
    timer_ = this->create_wall_timer(3s, publish_message);
  }

private:
  size_t count_ = 1;
  std::shared_ptr<diagnostic_msgs::msg::DiagnosticArray> msg_;
  std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus> msg_s;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  auto topic = std::string("chatter");
  auto node = std::make_shared<expected_pub>(topic);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
