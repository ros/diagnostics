// Copyright (c) 2024, 2025 Robert Bosch GmbH
//
// See the top-level LICENSE file for licensing terms.

#ifndef DIAGNOSTIC_TOPIC_MONITOR__GENERIC_TOPIC_MONITOR_HPP_
#define DIAGNOSTIC_TOPIC_MONITOR__GENERIC_TOPIC_MONITOR_HPP_

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
// Windows SDK defines ERROR as 0, which breaks scoped enum usage (e.g. CallbackReturn::ERROR)
#ifdef ERROR
#undef ERROR
#endif
#else
#include <unistd.h>
#endif

#include <limits>
#include <memory>
#include <regex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>       // NOLINT: upstream
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>  // NOLINT: upstream

#include "diagnostic_msgs/msg/diagnostic_array.hpp"  // NOLINT: upstream
#include "diagnostic_topic_monitor/activity_diagnostic_task.hpp"
#include "diagnostic_updater/publisher.hpp"         // NOLINT: upstream
#include "diagnostic_updater/update_functions.hpp"  // NOLINT: upstream

using namespace std::chrono_literals;  // NOLINT: build/namespaces

namespace diagnostic_topic_monitor
{
constexpr const char * TOPICS_PARAM_NAME = "topics";
constexpr const char * MIN_VALUES_PARAM_NAME = "min_values";
constexpr const char * MAX_VALUES_PARAM_NAME = "max_values";
constexpr const char * MONITOR_CONFIGURED_ONLY_PARAM_NAME =
  "monitor_configured_only";
constexpr const char * DIAG_PREFIX_PARAM_NAME = "diag_prefix";
typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
  CallbackReturn LCCBReturn;

/**
 * @brief Base abstract class for creating Topic Monitors.
 */
template<typename StatusType, typename StatusParamType>
class GenericTopicMonitor : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit GenericTopicMonitor(
    const std::string & node_name,
    rclcpp::NodeOptions options);
  ~GenericTopicMonitor() {}
  LCCBReturn on_configure(const rclcpp_lifecycle::State &) override;
  LCCBReturn on_activate(const rclcpp_lifecycle::State & state) override;
  LCCBReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  LCCBReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    return LCCBReturn::SUCCESS;
  }
  LCCBReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    return LCCBReturn::SUCCESS;
  }

protected:
  std::string get_prefixed_name(const std::string & topic_name) const;
  bool skip_topic(const std::string & topic_name);
  virtual void topic_cb(
    const std::string & topic_name,
    const std::shared_ptr<rclcpp::SerializedMessage> & msg) = 0;
  virtual void update_topic_subscriptions();
  virtual StatusParamType parse_params(const int index) = 0;

  std::vector<std::regex> hidden_topics_{
    std::regex("^/rosout$"), std::regex(".*/parameter_events"), std::regex(
      "^/diagnostics$"),
    std::regex(".*/transition_event"), std::regex("^/clock")};
  std::shared_ptr<rclcpp::TimerBase> timer_;
  std::shared_ptr<diagnostic_updater::Updater> updater_;
  std::unordered_map<std::string,
    std::shared_ptr<rclcpp::GenericSubscription>> subscribed_topics_;
  std::unordered_map<std::string, std::shared_ptr<StatusType>> topic_diag_map_;
  std::vector<std::string> topics_;
  std::set<std::string> known_topics_;
  std::vector<double> min_values_;
  std::vector<double> max_values_;
  bool monitor_configured_only_{true};
  std::string diag_prefix_;
};

// Implementation of GenericTopicMonitor methods
template<typename StatusType, typename StatusParamType>
inline GenericTopicMonitor<StatusType, StatusParamType>::GenericTopicMonitor(
  const std::string & node_name, rclcpp::NodeOptions options)
: rclcpp_lifecycle::LifecycleNode(node_name, options.allow_undeclared_parameters(
      true))
{
  auto desc = rcl_interfaces::msg::ParameterDescriptor{};
  desc.description = "Topics to specify expectations for";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  declare_parameter(TOPICS_PARAM_NAME, std::vector<std::string>(), desc);
  auto desc_min = rcl_interfaces::msg::ParameterDescriptor{};
  desc_min.description = "Minimum values for topic from 'topics'";
  desc_min.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
  declare_parameter(MIN_VALUES_PARAM_NAME, std::vector<double>(), desc_min);
  auto desc_max = rcl_interfaces::msg::ParameterDescriptor{};
  desc_max.description = "Maximum values for topic from 'topics'";
  desc_max.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
  declare_parameter(MAX_VALUES_PARAM_NAME, std::vector<double>(), desc_max);
  // by default, we include all topics
  declare_parameter(MONITOR_CONFIGURED_ONLY_PARAM_NAME, false);
  // prefix for diagnostics (for use with modules)
  declare_parameter(DIAG_PREFIX_PARAM_NAME, "");
}

template<typename StatusType, typename StatusParamType>
inline std::string
diagnostic_topic_monitor::GenericTopicMonitor<StatusType,
  StatusParamType>::get_prefixed_name(
  const std::string & topic_name) const
{
  if (topic_name.find(diag_prefix_) == 0) {
    return topic_name;
  }
  return diag_prefix_ + std::string("/") + topic_name;
}

template<typename StatusType, typename StatusParamType>
inline bool diagnostic_topic_monitor::GenericTopicMonitor<StatusType,
  StatusParamType>::skip_topic(
  const std::string & topic_name)
{
  // skip if we already subscribed
  if (subscribed_topics_.find(topic_name) != subscribed_topics_.end()) {
    RCLCPP_DEBUG(
      get_logger(), "Already subscribed to %s, skipping", topic_name.c_str());
    return true;
  }
  // if it's configured, we monitor it
  if (std::find(topics_.begin(), topics_.end(), topic_name) != topics_.end()) {
    RCLCPP_DEBUG(
      get_logger(), "Topic %s is configured, no skip", topic_name.c_str());
    return false;
  }
  // skip if we have seen this before and ignored it
  if (known_topics_.find(topic_name) != known_topics_.end()) {
    return true;
  }
  // check for matches against internal topic names
  for (const auto & re : hidden_topics_) {
    if (std::regex_match(topic_name, re)) {
      known_topics_.insert(topic_name);
      RCLCPP_DEBUG(
        get_logger(), "Topic %s matches ignore list, skip it",
        topic_name.c_str());
      return true;
    }
  }
  return monitor_configured_only_;
}

template<typename StatusType, typename StatusParamType>
inline void diagnostic_topic_monitor::GenericTopicMonitor<
  StatusType, StatusParamType>::update_topic_subscriptions()
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
    RCLCPP_DEBUG(
      get_logger(), "Starting to monitor topic %s",
      topic_name.c_str());
    std::shared_ptr<rclcpp::GenericSubscription> sub =
      this->create_generic_subscription(
      e.first, *e.second.begin(), rclcpp::QoS(10),
      [this, topic_name](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        this->topic_cb(topic_name, msg);
      });
    subscribed_topics_[topic_name] = sub;
  }
}

template<typename StatusType, typename StatusParamType>
inline LCCBReturn
diagnostic_topic_monitor::GenericTopicMonitor<StatusType,
  StatusParamType>::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(get_logger(), "Configuring");
  // configure the diagnostics
  topics_ = get_parameter(TOPICS_PARAM_NAME).as_string_array();
  min_values_ = get_parameter(MIN_VALUES_PARAM_NAME).as_double_array();
  max_values_ = get_parameter(MAX_VALUES_PARAM_NAME).as_double_array();
  if (topics_.size() != min_values_.size() ||
    topics_.size() != max_values_.size())
  {
    throw std::invalid_argument(
            "Topics and min/max_values must have same number of arguments");
  }
  monitor_configured_only_ =
    get_parameter(MONITOR_CONFIGURED_ONLY_PARAM_NAME).as_bool();
  diag_prefix_ = get_parameter(DIAG_PREFIX_PARAM_NAME).as_string();

  RCLCPP_DEBUG(
    get_logger(), "Done configuring for %ld topics, config only: %d",
    topic_diag_map_.size(),
    monitor_configured_only_);
  return LCCBReturn::SUCCESS;
}

template<typename StatusType, typename StatusParamType>
inline LCCBReturn
diagnostic_topic_monitor::GenericTopicMonitor<StatusType,
  StatusParamType>::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(get_logger(), "Activating");
  updater_ = std::make_shared<diagnostic_updater::Updater>(this);
  char HOSTNAME[1000];
  gethostname(HOSTNAME, 1000);
  updater_->setHardwareID(std::string(HOSTNAME));
  for (size_t i = 0; i < topics_.size(); ++i) {
    auto param = parse_params(i);
    auto diag = std::make_shared<StatusType>(
      param, get_prefixed_name(
        topics_[i]), get_clock());
    topic_diag_map_[topics_[i]] = diag;
    updater_->add(*diag);
  }
  // check existing topics
  try {
    update_topic_subscriptions();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      this->get_logger(), "Failure to update subscriptions: %s", ex.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
           CallbackReturn::ERROR;
  }
  timer_ =
    create_wall_timer(1s, [this]() {this->update_topic_subscriptions();});
  return LCCBReturn::SUCCESS;
}

template<typename StatusType, typename StatusParamType>
inline LCCBReturn
diagnostic_topic_monitor::GenericTopicMonitor<StatusType,
  StatusParamType>::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO_STREAM(get_logger(), "Deactivating GenericTopicMonitor");
  return LCCBReturn::SUCCESS;
}

}  // namespace diagnostic_topic_monitor

#endif  // DIAGNOSTIC_TOPIC_MONITOR__GENERIC_TOPIC_MONITOR_HPP_
