// Copyright (c) 2024, 2025 Robert Bosch GmbH
//
// See the top-level LICENSE file for licensing terms.

#include "diagnostic_topic_monitor/topic_age_monitor.hpp"

namespace diagnostic_topic_monitor
{

TopicAgeMonitor::TopicAgeMonitor(const std::string & node_name, rclcpp::NodeOptions options)
: GenericTopicMonitor(node_name, options)
{
}

TopicAgeMonitor::TopicAgeMonitor(rclcpp::NodeOptions options)
: GenericTopicMonitor("topic_age_monitor", options)
{
}

diagnostic_updater::TimeStampStatusParam diagnostic_topic_monitor::TopicAgeMonitor::parse_params(
  const int index)
{
  return diagnostic_updater::TimeStampStatusParam(min_values_[index], max_values_[index]);
}

void TopicAgeMonitor::topic_cb(
  const std::string & topic_name, const std::shared_ptr<rclcpp::SerializedMessage> & msg)
{
  std_msgs::msg::Header header;
  const auto diag = topic_diag_map_.find(topic_name);
  try {
    _header_serializer.deserialize_message(msg.get(), &header);
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      get_logger(), "Failed to deserialize message from topic %s, skipping it", topic_name.c_str());
    return;
  }

  if (diag != topic_diag_map_.end()) {
    diag->second->tick(header.stamp);
  } else {
    RCLCPP_WARN(get_logger(), "No diagnostic found for topic %s", topic_name.c_str());
  }
}

}  // namespace diagnostic_topic_monitor

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT: upstream
RCLCPP_COMPONENTS_REGISTER_NODE(diagnostic_topic_monitor::TopicAgeMonitor)
