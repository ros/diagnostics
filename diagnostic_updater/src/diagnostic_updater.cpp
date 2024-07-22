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

#include <diagnostic_updater/diagnostic_updater.hpp>

#include "rclcpp/rclcpp.hpp"

namespace diagnostic_updater
{
Updater::Updater(
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> base_interface,
  std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> clock_interface,
  std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> logging_interface,
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
  std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> timers_interface,
  std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> topics_interface, double period)
: verbose_(false),
  base_interface_(base_interface),
  timers_interface_(timers_interface),
  clock_(clock_interface->get_clock()),
  period_(rclcpp::Duration::from_seconds(period)),
  publisher_(rclcpp::create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      topics_interface, "/diagnostics", 1)),
  logger_(logging_interface->get_logger()),
  node_name_(base_interface->get_name()),
  warn_nohwid_done_(false)
{
  constexpr const char * period_param_name = "diagnostic_updater.period";
  rclcpp::ParameterValue period_param;
  if (parameters_interface->has_parameter(period_param_name)) {
    period_param = parameters_interface->get_parameter(period_param_name).get_parameter_value();
  } else {
    period_param =
      parameters_interface->declare_parameter(period_param_name, rclcpp::ParameterValue(period));
  }
  period = period_param.get<double>();
  period_ = rclcpp::Duration::from_seconds(period);

  reset_timer();

  constexpr const char * use_fqn_param_name = "diagnostic_updater.use_fqn";
  rclcpp::ParameterValue use_fqn_param;
  if (parameters_interface->has_parameter(use_fqn_param_name)) {
    use_fqn_param = parameters_interface->get_parameter(use_fqn_param_name).get_parameter_value();
  } else {
    use_fqn_param =
      parameters_interface->declare_parameter(use_fqn_param_name, rclcpp::ParameterValue(false));
  }
  node_name_ = use_fqn_param.get<bool>() ? base_interface->get_fully_qualified_name() :
    base_interface->get_name();
}

void Updater::broadcast(unsigned char lvl, const std::string msg)
{
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> status_vec;

  const std::vector<DiagnosticTaskInternal> & tasks = getTasks();
  for (std::vector<DiagnosticTaskInternal>::const_iterator iter = tasks.begin();
    iter != tasks.end(); iter++)
  {
    diagnostic_updater::DiagnosticStatusWrapper status;

    status.name = iter->getName();
    status.summary(lvl, msg);

    status_vec.push_back(status);
  }

  publish(status_vec);
}

void Updater::setHardwareIDf(const char * format, ...)
{
  va_list va;
  const int kBufferSize = 1000;
  char buff[kBufferSize];  // @todo This could be done more elegantly.
  va_start(va, format);
  if (vsnprintf(buff, kBufferSize, format, va) >= kBufferSize) {
    RCLCPP_DEBUG(logger_, "Really long string in diagnostic_updater::setHardwareIDf.");
  }
  hwid_ = std::string(buff);
  va_end(va);
}

void Updater::reset_timer()
{
  update_timer_ = rclcpp::create_timer(
    base_interface_, timers_interface_, clock_, period_, std::bind(&Updater::update, this));
}

void Updater::update()
{
  if (rclcpp::ok(base_interface_->get_context())) {
    bool warn_nohwid = hwid_.empty();

    std::vector<diagnostic_msgs::msg::DiagnosticStatus> status_vec;

    std::unique_lock<std::mutex> lock(
      lock_);  // Make sure no adds happen while we are processing here.
    const std::vector<DiagnosticTaskInternal> & tasks = getTasks();
    for (std::vector<DiagnosticTaskInternal>::const_iterator iter = tasks.begin();
      iter != tasks.end(); iter++)
    {
      diagnostic_updater::DiagnosticStatusWrapper status;

      status.name = iter->getName();
      status.level = 2;
      status.message = "No message was set";
      status.hardware_id = hwid_;

      iter->run(status);

      status_vec.push_back(status);

      if (status.level) {
        warn_nohwid = false;
      }

      if (verbose_ && status.level) {
        RCLCPP_WARN(
          logger_, "Non-zero diagnostic status. Name: '%s', status %i: '%s'", status.name.c_str(),
          status.level, status.message.c_str());
      }
    }

    if (warn_nohwid && !warn_nohwid_done_) {
      std::string error_msg = "diagnostic_updater: No HW_ID was set.";
      error_msg += " This is probably a bug. Please report it.";
      error_msg += " For devices that do not have a HW_ID, set this value to 'none'.";
      error_msg += " This warning only occurs once all diagnostics are OK.";
      error_msg += " It is okay to wait until the device is open before calling setHardwareID.";
      RCLCPP_WARN(logger_, "%s", error_msg.c_str());
      warn_nohwid_done_ = true;
    }

    publish(status_vec);
  }
}

void Updater::publish(diagnostic_msgs::msg::DiagnosticStatus & stat)
{
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> status_vec;
  status_vec.push_back(stat);
  publish(status_vec);
}

void Updater::publish(std::vector<diagnostic_msgs::msg::DiagnosticStatus> & status_vec)
{
  for (std::vector<diagnostic_msgs::msg::DiagnosticStatus>::iterator iter = status_vec.begin();
    iter != status_vec.end(); iter++)
  {
    iter->name = node_name_ + std::string(": ") + iter->name;
  }
  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.status = status_vec;
  msg.header.stamp = clock_->now();
  publisher_->publish(msg);
}

void Updater::addedTaskCallback(DiagnosticTaskInternal & task)
{
  DiagnosticStatusWrapper stat;
  stat.name = task.getName();
  stat.summary(0, "Node starting up");
  publish(stat);
}
}  // namespace diagnostic_updater
