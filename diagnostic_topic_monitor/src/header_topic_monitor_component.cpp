//  Copyright 2024 Robert Bosch GmbH and its subsidiaries
//  All rights reserved, also regarding any disposal, exploitation,
//  reproduction, editing, distribution, as well as in the event of applications
//  for industrial property rights.

#include <unistd.h>
#include <regex>
#include <set>
#include <string>
#include <vector>
#include <unordered_map>
#include <limits>
#include <memory>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>   // NOLINT: upstream
#include <diagnostic_updater/diagnostic_updater.hpp>  // NOLINT: upstream
#include <diagnostic_updater/publisher.hpp>           // NOLINT: upstream
#include <rclcpp_lifecycle/lifecycle_node.hpp>        // NOLINT: upstream
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>   // NOLINT: upstream
#include "rclcpp/serialization.hpp"
#include "update_functions.hpp"

using namespace std::chrono_literals;

namespace diagnostic_topic_monitor
{
constexpr const char * TOPICS_PARAM_NAME = "topics";
constexpr const char * MIN_DELAYS_PARAM_NAME = "min_delays";
constexpr const char * MAX_DELAYS_PARAM_NAME = "max_delays";
constexpr const char * MONITOR_CONFIGURED_ONLY_PARAM_NAME = "monitor_configured_only";
constexpr const char * DIAG_PREFIX_PARAM_NAME = "diag_prefix";


using namespace std::chrono_literals;
class HeaderTopicMonitor : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit HeaderTopicMonitor(const std::string & node_name, rclcpp::NodeOptions options)
  : rclcpp_lifecycle::LifecycleNode(node_name, options.allow_undeclared_parameters(true))
  {
    auto desc = rcl_interfaces::msg::ParameterDescriptor{};
    desc.description = "Topics to specify expectations for";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
    declare_parameter(TOPICS_PARAM_NAME, std::vector<std::string>(), desc);
    auto desc_min = rcl_interfaces::msg::ParameterDescriptor{};
    desc_min.description = "Minimum delay for topic from 'topics'";
    desc_min.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
    declare_parameter(MIN_DELAYS_PARAM_NAME, std::vector<double>(), desc_min);
    auto desc_max = rcl_interfaces::msg::ParameterDescriptor{};
    desc_max.description = "Maximum delay for topic from 'topics'";
    desc_max.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
    declare_parameter(MAX_DELAYS_PARAM_NAME, std::vector<double>(), desc_max);
    // by default, we include all topics
    declare_parameter(MONITOR_CONFIGURED_ONLY_PARAM_NAME, false);
    // prefix for diagnostics (for use with modules)
    declare_parameter(DIAG_PREFIX_PARAM_NAME, "");
  }
  explicit HeaderTopicMonitor(const rclcpp::NodeOptions & options)
  : HeaderTopicMonitor("header_topic_monitor", options)
  {
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &) override
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  void topic_cb(
    const std::string & topic_name,
    const std::shared_ptr<rclcpp::SerializedMessage> & msg)
  {
    std_msgs::msg::Header header;
    const auto diag = topic_diag_map_.find(topic_name);
    _header_serializer.deserialize_message(msg.get(), &header);

    if (diag != topic_diag_map_.end()) {
      diag->second->tick(header.stamp);
    } else {
      RCLCPP_WARN(get_logger(), "No diagnostic found for topic %s", topic_name.c_str());
    }
  }

protected:
  void update_topic_subscriptions();
  bool skip_topic(const std::string & name);
  std::string get_prefixed_name(const std::string & topic_name) const;

private:
  std::vector<std::regex> hidden_topics_{std::regex("^/rosout$"), std::regex(".*/parameter_events"),
    std::regex("^/diagnostics$"), std::regex(".*/transition_event"), std::regex("^/clock")};
  std::shared_ptr<rclcpp::TimerBase> timer_;
  std::shared_ptr<diagnostic_updater::Updater> updater_;
  std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericSubscription>> subscribed_topics_;
  std::set<std::string> known_topics_;
  std::unordered_map<std::string,
    std::shared_ptr<topic_monitor::TimeStampStatus>> topic_diag_map_;
  std::vector<std::string> topics_;
  std::vector<double> min_delays;
  std::vector<double> max_delays;
  bool monitor_configured_only_{true};
  std::string diag_prefix_;
  rclcpp::Serialization<std_msgs::msg::Header> _header_serializer;
};

std::string HeaderTopicMonitor::get_prefixed_name(const std::string & topic_name) const
{
  if (topic_name.find(diag_prefix_) == 0) {
    return topic_name;
  }
  return diag_prefix_ + std::string("/") + topic_name;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HeaderTopicMonitor::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(get_logger(), "Configuring");
  // configure the diagnostics
  topics_ = get_parameter(TOPICS_PARAM_NAME).as_string_array();
  min_delays = get_parameter(MIN_DELAYS_PARAM_NAME).as_double_array();
  max_delays = get_parameter(MAX_DELAYS_PARAM_NAME).as_double_array();
  if (topics_.size() != min_delays.size() || topics_.size() != max_delays.size()) {
    throw std::invalid_argument("Topics and min/max_delays must have same number of arguments");
  }
  monitor_configured_only_ = get_parameter(MONITOR_CONFIGURED_ONLY_PARAM_NAME).as_bool();
  diag_prefix_ = get_parameter(DIAG_PREFIX_PARAM_NAME).as_string();

  RCLCPP_DEBUG(
    get_logger(), "Done configuring for %ld topics, config only: %d", topic_diag_map_.size(),
    monitor_configured_only_);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HeaderTopicMonitor::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(get_logger(), "Activating");
  updater_ = std::make_shared<diagnostic_updater::Updater>(this);
  char HOSTNAME[1000];
  gethostname(HOSTNAME, 1000);
  updater_->setHardwareID(std::string(HOSTNAME));
  for (size_t i = 0; i < topics_.size(); ++i) {
    topic_monitor::TimeStampStatusParam param(min_delays[i], max_delays[i]);
    auto diag = std::make_shared<topic_monitor::TimeStampStatus>(
      param, get_prefixed_name(topics_[i]), get_clock());
    topic_diag_map_[topics_[i]] = diag;
    updater_->add(*diag);
  }
  // check existing topics
  try {
    update_topic_subscriptions();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(this->get_logger(), "Failure to update subscriptions: %s", ex.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  timer_ = create_wall_timer(1s, [this]() {this->update_topic_subscriptions();});
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HeaderTopicMonitor::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(get_logger(), "Deactivating");
  timer_.reset();
  subscribed_topics_.clear();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool HeaderTopicMonitor::skip_topic(const std::string & topic_name)
{
  // skip if we already subscribed
  if (subscribed_topics_.find(topic_name) != subscribed_topics_.end()) {
    RCLCPP_DEBUG(get_logger(), "Already subscribed to %s, skipping", topic_name.c_str());
    return true;
  }
  // if it's configured, we monitor it
  if (std::find(topics_.begin(), topics_.end(), topic_name) != topics_.end()) {
    RCLCPP_DEBUG(get_logger(), "Topic %s is configured, no skip", topic_name.c_str());
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
      RCLCPP_DEBUG(get_logger(), "Topic %s matches ignore list, skip it", topic_name.c_str());
      return true;
    }
  }
  return monitor_configured_only_;
}

void HeaderTopicMonitor::update_topic_subscriptions()
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
  }
  RCLCPP_DEBUG(
    get_logger(), "Done updating topic list. Found %ld, hidden %ld, now monitoring %ld",
    topics.size(),
    known_topics_.size(), subscribed_topics_.size());
}
}  // namespace diagnostic_topic_monitor

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT: upstream
RCLCPP_COMPONENTS_REGISTER_NODE(diagnostic_topic_monitor::HeaderTopicMonitor)
