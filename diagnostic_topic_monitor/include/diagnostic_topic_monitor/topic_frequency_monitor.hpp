// Copyright (c) 2024, 2025 Robert Bosch GmbH
//
// See the top-level LICENSE file for licensing terms.

#ifndef DIAGNOSTIC_TOPIC_MONITOR__TOPIC_FREQUENCY_MONITOR_HPP_
#define DIAGNOSTIC_TOPIC_MONITOR__TOPIC_FREQUENCY_MONITOR_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "diagnostic_topic_monitor/generic_topic_monitor.hpp"

namespace diagnostic_topic_monitor
{

class TopicFrequencyMonitor final
  : public GenericTopicMonitor<
    diagnostic_updater::FrequencyStatus,
    diagnostic_updater::FrequencyStatusParam>
{
public:
  TopicFrequencyMonitor(
    const std::string & node_name,
    rclcpp::NodeOptions options);
  TopicFrequencyMonitor(rclcpp::NodeOptions options = rclcpp::NodeOptions());
  ~TopicFrequencyMonitor() override = default;

  void topic_cb(
    const std::string & topic_name,
    const std::shared_ptr<rclcpp::SerializedMessage> & msg) override;
  diagnostic_updater::FrequencyStatusParam parse_params(const int index)
  override;
  void update_topic_subscriptions() override;

private:
  std::unordered_map<std::string,
    std::shared_ptr<ActivityDiagnosticTask>> fallback_topic_diag_map_;
};

}  // namespace diagnostic_topic_monitor

#endif  // DIAGNOSTIC_TOPIC_MONITOR__TOPIC_FREQUENCY_MONITOR_HPP_
