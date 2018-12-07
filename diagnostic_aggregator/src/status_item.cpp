// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <diagnostic_aggregator/status_item.h>

using namespace diagnostic_aggregator;
using namespace std;
/*rclcpp::Clock ros_clock(RCL_ROS_TIME);*/

StatusItem::StatusItem(const diagnostic_msgs::msg::DiagnosticStatus *status)
{
  level_ = valToLevel(status->level);
  name_ = status->name;
  message_ = status->message;
  hw_id_ = status->hardware_id;
  values_ = status->values;
  
  output_name_ = getOutputName(name_);
  rclcpp::Clock ros_clock(RCL_ROS_TIME);
  update_time_ = ros_clock.now();

  /*builtin_interfaces::msg::Time update_time_ = ros_clock.now();
  update_time_ = ros::Time::now();*/
}

StatusItem::StatusItem(const string item_name, const string message, const DiagnosticLevel level)
{
  name_ = item_name;
  message_ = message;
  level_ = level;
  hw_id_ = "";

  cout<< "StatusItem name is =  " << name_ <<endl;  
  output_name_ = getOutputName(name_);
  rclcpp::Clock ros_clock(RCL_ROS_TIME);
   update_time_ = ros_clock.now();
 /* builtin_interfaces::msg::Time update_time_ = ros_clock.now();
  update_time_ = ros::Time::now();*/
}

StatusItem::~StatusItem() {}

bool StatusItem::update(const diagnostic_msgs::msg::DiagnosticStatus *status)
{
  if (name_ != status->name)
  {
    ROS_ERROR("Incorrect name when updating StatusItem. Expected %s, got %s", name_.c_str(), status->name.c_str());
    return false;
  }

  rclcpp::Clock ros_clock(RCL_ROS_TIME);
  rclcpp::Time update_time_now_ = ros_clock.now();
  /*builtin_interfaces::msg::Time update_time_now_ = ros_clock.now();
  builtin_interfaces::msg::Duration update_interval_now = (update_time_now_ - update_time_)
  builtin_interfaces::msg::Duration update_interval = update_interval_now.sec;*/
  rclcpp::Duration update_interval_now =  update_time_now_ - update_time_ ;
  double update_interval = (update_interval_now.nanoseconds())*1e-9;
  if ( update_interval < 0 )
    ROS_WARN("StatusItem is being updated with older data. Negative update time: %f", update_interval);

  level_ = valToLevel(status->level);
  message_ = status->message;
  hw_id_ = status->hardware_id;
  values_ = status->values;
  rclcpp::Time update_time_ = ros_clock.now();
 
 /* builtin_interfaces::msg::Time update_time_ = ros_clock.now();
  update_time_ = ros::Time::now();*/

  return true;
}

std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus> StatusItem::toStatusMsg(const std::string &path, bool stale) const
{
  std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus> status(new diagnostic_msgs::msg::DiagnosticStatus());

  if (path == "/")
    status->name = "/" + output_name_;
  else
    status->name = path + "/" + output_name_;

  status->level = level_;
  status->message = message_;
  status->hardware_id = hw_id_;
  status->values = values_;

  if (stale)
    status->level = Level_Stale;

  return status;
}

