#include "diagnostic_topic_monitor/topic_frquency_monitor.hpp"

namespace diagnostic_topic_monitor
{

TopicFrequencyMonitor::TopicFrequencyMonitor(
  const std::string & node_name, rclcpp::NodeOptions options)
: GenericTopicMonitor(node_name, options)
{
}

TopicFrequencyMonitor::TopicFrequencyMonitor(rclcpp::NodeOptions options)
: GenericTopicMonitor("topic_frequency_monitor", options)
{
}

diagnostic_updater::FrequencyStatusParam
diagnostic_topic_monitor::TopicFrequencyMonitor::parse_params(const int index)
{
  return diagnostic_updater::FrequencyStatusParam(&min_values_[index], &max_values_[index]);
}

void TopicFrequencyMonitor::topic_cb(
  const std::string & topic_name, const std::shared_ptr<rclcpp::SerializedMessage> &)
{
  const auto diag = topic_diag_map_.find(topic_name);
  if (diag != topic_diag_map_.end()) {
    diag->second->tick();
  } else {
    const auto fallback_diag = fallback_topic_diag_map_.find(topic_name);
    if (fallback_diag != fallback_topic_diag_map_.end()) {
      fallback_diag->second->tick();
    } else {
      RCLCPP_WARN(get_logger(), "No diagnostic found for topic %s", topic_name.c_str());
    }
  }
}

void diagnostic_topic_monitor::TopicFrequencyMonitor::update_topic_subscriptions()
{
  RCLCPP_DEBUG(
    get_logger(), "Examining topic list for changes, currently monitoring %ld topics",
    subscribed_topics_.size());
  const auto topics = this->get_topic_names_and_types();
  for (const auto & e : topics) {
    const std::string topic_name = e.first;
    if (skip_topic(topic_name)) {
      continue;
    }
    // subscribe with statistics enabled
    RCLCPP_DEBUG(get_logger(), "Starting to monitor topic %s", topic_name.c_str());
    std::shared_ptr<rclcpp::GenericSubscription> sub = this->create_generic_subscription(
      e.first, *e.second.begin(), rclcpp::QoS(10),
      [this, topic_name](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        this->topic_cb(topic_name, msg);
      });
    subscribed_topics_[topic_name] = sub;
    // add a default diagnostic if none is configured
    if (topic_diag_map_.find(topic_name) == topic_diag_map_.end()) {
      auto fallback_diag =
        std::make_shared<ActivityDiagnosticTask>(get_prefixed_name(topic_name), get_clock());
      fallback_topic_diag_map_[topic_name] = fallback_diag;
      updater_->add(*fallback_diag);
    }
  }
  RCLCPP_DEBUG(
    get_logger(), "Done updating topic list. Found %ld, hidden %ld, now monitoring %ld",
    topics.size(), known_topics_.size(), subscribed_topics_.size());
}

}  // namespace diagnostic_topic_monitor

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT: upstream
RCLCPP_COMPONENTS_REGISTER_NODE(diagnostic_topic_monitor::TopicFrequencyMonitor)
