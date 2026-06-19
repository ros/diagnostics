// Copyright (c) 2024, 2025 Robert Bosch GmbH
//
// See the top-level LICENSE file for licensing terms.

#ifndef DIAGNOSTIC_TOPIC_MONITOR__TOPIC_AGE_MONITOR_HPP_
#define DIAGNOSTIC_TOPIC_MONITOR__TOPIC_AGE_MONITOR_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "diagnostic_topic_monitor/generic_topic_monitor.hpp"

namespace diagnostic_topic_monitor
{

class TopicAgeMonitor final
  : public GenericTopicMonitor<
    diagnostic_updater::TimeStampStatus,
    diagnostic_updater::TimeStampStatusParam>
{
public:
  TopicAgeMonitor(const std::string & node_name, rclcpp::NodeOptions options);
  TopicAgeMonitor(rclcpp::NodeOptions options = rclcpp::NodeOptions());
  ~TopicAgeMonitor() override = default;

  void topic_cb(
    const std::string & topic_name,
    const std::shared_ptr<rclcpp::SerializedMessage> & msg) override;
  diagnostic_updater::TimeStampStatusParam parse_params(const int index)
  override;

private:
  rclcpp::Serialization<std_msgs::msg::Header> _header_serializer;
};

}  // namespace diagnostic_topic_monitor

#endif  // DIAGNOSTIC_TOPIC_MONITOR__TOPIC_AGE_MONITOR_HPP_
