/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/**!< \author Kevin Watts */

#include "diagnostic_aggregator/status_item.hpp"

#include <memory>
#include <string>

namespace diagnostic_aggregator
{
using std::string;

using rclcpp::get_logger;

StatusItem::StatusItem(const diagnostic_msgs::msg::DiagnosticStatus * status)
: clock_(new rclcpp::Clock())
{
  level_ = valToLevel(status->level);
  name_ = status->name;
  message_ = status->message;
  hw_id_ = status->hardware_id;
  values_ = status->values;

  output_name_ = getOutputName(name_);

  update_time_ = clock_->now();
}

StatusItem::StatusItem(const string item_name, const string message, const DiagnosticLevel level)
: clock_(new rclcpp::Clock())
{
  RCLCPP_DEBUG(rclcpp::get_logger("StatusItem"), "StatusItem constructor from string");
  name_ = item_name;
  message_ = message;
  level_ = level;
  hw_id_ = "";

  output_name_ = getOutputName(name_);

  update_time_ = clock_->now();
  RCLCPP_DEBUG(rclcpp::get_logger("StatusItem"), "StatusItem constructor from string");
}

StatusItem::~StatusItem() {}

bool StatusItem::update(const diagnostic_msgs::msg::DiagnosticStatus * status)
{
  if (name_ != status->name) {
    RCLCPP_ERROR(
      get_logger("status_item"), "Incorrect name when updating StatusItem. Expected %s, got %s",
      name_.c_str(), status->name.c_str());
    return false;
  }

  double update_interval = (clock_->now() - update_time_).seconds();
  if (update_interval < 0) {
    RCLCPP_WARN(
      get_logger("status_item"),
      "StatusItem is being updated with older data. Negative update time: %f", update_interval);
  }

  level_ = valToLevel(status->level);
  message_ = status->message;
  hw_id_ = status->hardware_id;
  values_ = status->values;

  update_time_ = clock_->now();

  return true;
}

std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus> StatusItem::toStatusMsg(
  const std::string & path, bool stale) const
{
  std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus> status(
    new diagnostic_msgs::msg::DiagnosticStatus());

  if (path == "/") {
    status->name = "/" + output_name_;
  } else {
    status->name = path + "/" + output_name_;
  }

  status->level = level_;
  status->message = message_;
  status->hardware_id = hw_id_;
  status->values = values_;

  if (stale) {
    status->level = Level_Stale;
  }

  return status;
}

}  // namespace diagnostic_aggregator
