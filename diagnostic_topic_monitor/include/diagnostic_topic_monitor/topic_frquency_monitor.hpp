#ifndef DIAGNOSTIC_TOPIC_MONITOR_TOPIC_FREQUENCY_MONITOR_HPP
#define DIAGNOSTIC_TOPIC_MONITOR_TOPIC_FREQUENCY_MONITOR_HPP

#include "diagnostic_topic_monitor/generic_topic_monitor.hpp"

namespace diagnostic_topic_monitor
{

class TopicFrequencyMonitor final
  : public GenericTopicMonitor<
    diagnostic_updater::FrequencyStatus, diagnostic_updater::FrequencyStatusParam>
{
public:
  TopicFrequencyMonitor(const std::string & node_name, rclcpp::NodeOptions options);
  TopicFrequencyMonitor(rclcpp::NodeOptions options = rclcpp::NodeOptions());
  ~TopicFrequencyMonitor() override = default;

  void topic_cb(
    const std::string & topic_name,
    const std::shared_ptr<rclcpp::SerializedMessage> & msg) override;
  diagnostic_updater::FrequencyStatusParam parse_params(const int index) override;
  void update_topic_subscriptions() override;

private:
  std::unordered_map<std::string, std::shared_ptr<ActivityDiagnosticTask>> fallback_topic_diag_map_;
};

}  // namespace diagnostic_topic_monitor

#endif  // DIAGNOSTIC_TOPIC_MONITOR_TOPIC_FREQUENCY_MONITOR_HPP
