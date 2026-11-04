// Copyright (c) 2024, 2025 Robert Bosch GmbH
//
// See the top-level LICENSE file for licensing terms.

#ifndef DIAGNOSTIC_TOPIC_MONITOR__ACTIVITY_DIAGNOSTIC_TASK_HPP_
#define DIAGNOSTIC_TOPIC_MONITOR__ACTIVITY_DIAGNOSTIC_TASK_HPP_

#include <string>
#include <limits>

#include <diagnostic_updater/diagnostic_updater.hpp>  // NOLINT: upstream

using namespace std::chrono_literals;  // NOLINT: build/namespaces

namespace diagnostic_topic_monitor
{

/**
 * @brief Very simply DiagnosticTask that just checks whether there has been _any_
 * kind of activity during the last period
 */
class ActivityDiagnosticTask : public diagnostic_updater::DiagnosticTask
{
public:
  ActivityDiagnosticTask(
    const std::string & name, rclcpp::Clock::SharedPtr clock,
    std::chrono::duration<double> warning_threshold = 1.1s)
  : diagnostic_updater::DiagnosticTask(name),
    clock_(clock),
    last_tick_(0),
    warning_threshold_(warning_threshold)
  {
  }
  virtual void tick() {last_tick_ = clock_->now();}
  void run(diagnostic_updater::DiagnosticStatusWrapper & stat) override
  {
    stat.name = getName();
    if (last_tick_.seconds() == 0) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      stat.message = "No data received, yet";
      stat.addf("period", "%f", std::numeric_limits<double>::max());
    } else {
      const auto currentTime = clock_->now();
      // more than 1s without tick is considered warning
      const auto elapsed = currentTime - last_tick_;
      stat.addf("period", "%f", elapsed.seconds());
      if (elapsed <= rclcpp::Duration(warning_threshold_)) {
        stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        stat.message = "OK";
      } else {
        stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        stat.message = "Last data received more than 1s ago";
      }
    }
  }

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time last_tick_;
  std::chrono::duration<double> warning_threshold_;
};

}  // namespace diagnostic_topic_monitor

#endif  // DIAGNOSTIC_TOPIC_MONITOR__ACTIVITY_DIAGNOSTIC_TASK_HPP_
