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

/**! \author Kevin Watts */
/**! \author Arne Nordmann */

#include "diagnostic_aggregator/aggregator.hpp"

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace diagnostic_aggregator
{
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;

/**
 * @todo(anordman): make aggregator a lifecycle node.
 */
Aggregator::Aggregator()
: Aggregator(rclcpp::NodeOptions()) {}

Aggregator::Aggregator(rclcpp::NodeOptions options)
: n_(std::make_shared<rclcpp::Node>(
      "analyzers", "",
      options.allow_undeclared_parameters(true).
      automatically_declare_parameters_from_overrides(true))),
  logger_(rclcpp::get_logger("Aggregator")),
  pub_rate_(1.0),
  history_depth_(1000),
  clock_(n_->get_clock()),
  base_path_(""),
  critical_(false),
  publish_values_(true),
  last_top_level_state_(DiagnosticStatus::STALE)
{
  RCLCPP_DEBUG(logger_, "constructor");
  initAnalyzers();

  diag_sub_ = n_->create_subscription<DiagnosticArray>(
    "/diagnostics", rclcpp::SystemDefaultsQoS().keep_last(history_depth_),
    std::bind(&Aggregator::diagCallback, this, _1));
  agg_pub_ = n_->create_publisher<DiagnosticArray>("/diagnostics_agg", 1);
  toplevel_state_pub_ =
    n_->create_publisher<DiagnosticStatus>("/diagnostics_toplevel_state", 1);

  int publish_rate_ms = 1000 / pub_rate_;
  publish_timer_ = n_->create_wall_timer(
    std::chrono::milliseconds(publish_rate_ms),
    std::bind(&Aggregator::publishData, this));

  param_cb_handle_ = n_->add_on_set_parameters_callback(
    std::bind(&Aggregator::onParametersSet, this, _1));
}

rcl_interfaces::msg::SetParametersResult Aggregator::onParametersSet(
  const std::vector<rclcpp::Parameter> & parameters)
{
  // Check if any of the incoming parameters are new. If so, flag for reinitialization.
  // The method publishData() will pick it up on the next publish cycle and call initAnalyzers().
  for (const auto & p : parameters) {
    if (!n_->has_parameter(p.get_name())) {
      reinit_needed_.store(true);
      break;
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void Aggregator::initAnalyzers()
{
  bool other_as_errors = false;

  std::map<std::string, rclcpp::Parameter> parameters;
  if (!n_->get_parameters("", parameters)) {
    RCLCPP_ERROR(logger_, "Couldn't retrieve parameters.");
  }
  RCLCPP_DEBUG(logger_, "Retrieved %zu parameter(s).", parameters.size());

  for (const auto & param : parameters) {
    if (param.first.compare("pub_rate") == 0) {
      pub_rate_ = param.second.as_double();
    } else if (param.first.compare("path") == 0) {
      // Leading slash when path is not empty
      if (!param.second.as_string().empty()) {
        base_path_.append("/");
      }
      base_path_.append(param.second.as_string());
    } else if (param.first.compare("other_as_errors") == 0) {
      other_as_errors = param.second.as_bool();
    } else if (param.first.compare("history_depth") == 0) {
      history_depth_ = param.second.as_int();
    } else if (param.first.compare("critical") == 0) {
      critical_ = param.second.as_bool();
    } else if (param.first.compare("publish_values") == 0) {
      publish_values_ = param.second.as_bool();
    }
  }
  RCLCPP_DEBUG(logger_, "Aggregator publication rate configured to: %f", pub_rate_);
  RCLCPP_DEBUG(logger_, "Aggregator base path configured to: %s", base_path_.c_str());
  RCLCPP_DEBUG(
    logger_, "Aggregator other_as_errors configured to: %s", (other_as_errors ? "true" : "false"));
  RCLCPP_DEBUG(
    logger_, "Aggregator critical publisher configured to: %s", (critical_ ? "true" : "false"));
  RCLCPP_DEBUG(
    logger_, "Aggregator publish_values configured to: %s", (publish_values_ ? "true" : "false"));

  {  // lock the mutex while analyzer_group_ and other_analyzer_ are being updated
    std::lock_guard<std::mutex> lock(mutex_);
    analyzer_group_ = std::make_unique<AnalyzerGroup>();
    if (!analyzer_group_->init(base_path_, "", n_)) {
      RCLCPP_ERROR(logger_, "Analyzer group for diagnostic aggregator failed to initialize!");
    }

    // Last analyzer handles remaining data
    other_analyzer_ = std::make_unique<OtherAnalyzer>(other_as_errors);
    other_analyzer_->init(base_path_);  // This always returns true
  }
}

void Aggregator::checkTimestamp(const DiagnosticArray::SharedPtr diag_msg)
{
  RCLCPP_DEBUG(logger_, "checkTimestamp()");
  if (diag_msg->header.stamp.sec != 0) {
    return;
  }

  std::string stamp_warn = "No timestamp set for diagnostic message. Message names: ";
  std::vector<DiagnosticStatus>::const_iterator it;
  for (it = diag_msg->status.begin(); it != diag_msg->status.end(); ++it) {
    if (it != diag_msg->status.begin()) {
      stamp_warn += ", ";
    }
    stamp_warn += it->name;
  }

  auto result = ros_warnings_.insert(stamp_warn);
  if (result.second) {
    RCLCPP_WARN(logger_, "%s", stamp_warn.c_str());
  }
}

void Aggregator::diagCallback(const DiagnosticArray::SharedPtr diag_msg)
{
  RCLCPP_DEBUG(logger_, "diagCallback()");
  checkTimestamp(diag_msg);

  bool analyzed = false;
  bool immediate_report = false;
  {  // lock the whole loop to ensure nothing in the analyzer group changes during it.
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto j = 0u; j < diag_msg->status.size(); ++j) {
      analyzed = false;
      auto item = std::make_shared<StatusItem>(&diag_msg->status[j]);

      if (analyzer_group_->match(item->getName())) {
        analyzed = analyzer_group_->analyze(item);
      }

      if (!analyzed) {
        other_analyzer_->analyze(item);
      }

      // In case there is a degraded state, publish immediately
      if (critical_ && item->getLevel() > last_top_level_state_) {
        immediate_report = true;
      }
    }
  }

  if (immediate_report) {
    publishData();
  }
}

Aggregator::~Aggregator()
{
  RCLCPP_DEBUG(logger_, "destructor");
}

void Aggregator::publishData()
{
  RCLCPP_DEBUG(logger_, "publishData()");

  // Check if reinitialization is needed because new parameters have been set.
  if (reinit_needed_.exchange(false)) {
    base_path_ = "";
    initAnalyzers();
  }

  DiagnosticArray diag_array;
  DiagnosticStatus diag_toplevel_state;
  diag_toplevel_state.name = "toplevel_state";
  diag_toplevel_state.level = DiagnosticStatus::STALE;
  int max_level = -1;
  uint8_t max_level_without_stale = 0;
  int non_ok_status_depth = 0;
  std::shared_ptr<DiagnosticStatus> msg_to_report;

  std::vector<std::shared_ptr<DiagnosticStatus>> processed;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    processed = analyzer_group_->report();
  }
  for (const auto & msg : processed) {
    diag_array.status.push_back(*msg);
    const auto depth = std::count(msg->name.begin(), msg->name.end(), '/');

    if (msg->level > max_level) {
      max_level = msg->level;
      non_ok_status_depth = depth;
      msg_to_report = msg;
    }
    if (msg->level == max_level && depth > non_ok_status_depth) {
      // On non okay diagnostics also copy the deepest message to toplevel state
      non_ok_status_depth = depth;
      msg_to_report = msg;
    }
    if (
      msg->level > max_level_without_stale &&
      msg->level != diagnostic_msgs::msg::DiagnosticStatus::STALE)
    {
      max_level_without_stale = msg->level;
    }
  }

  std::vector<std::shared_ptr<DiagnosticStatus>> processed_other =
    other_analyzer_->report();
  for (const auto & msg : processed_other) {
    diag_array.status.push_back(*msg);
    const auto depth = std::count(msg->name.begin(), msg->name.end(), '/');

    if (msg->level > max_level) {
      max_level = msg->level;
      non_ok_status_depth = depth;
      msg_to_report = msg;
    }
    if (msg->level == max_level && depth > non_ok_status_depth) {
      // On non okay diagnostics also copy the deepest message to toplevel state
      non_ok_status_depth = depth;
      msg_to_report = msg;
    }
    if (
      msg->level > max_level_without_stale &&
      msg->level != diagnostic_msgs::msg::DiagnosticStatus::STALE)
    {
      max_level_without_stale = msg->level;
    }
  }

  // When a non-ok item was found, surface the offender via message/hardware_id/values
  // but keep name stable as "toplevel_state" to avoid breaking downstream consumers
  if (max_level > DiagnosticStatus::OK && msg_to_report) {
    diag_toplevel_state.message = msg_to_report->name + ": " + msg_to_report->message;
    diag_toplevel_state.hardware_id = msg_to_report->hardware_id;
    diag_toplevel_state.values = msg_to_report->values;
  }

  // If "publish_values" is false, clear all values
  if (!publish_values_) {
    for (auto & status : diag_array.status) {
      status.values.clear();
    }
  }

  diag_array.header.stamp = clock_->now();
  agg_pub_->publish(diag_array);

  if (max_level_without_stale > DiagnosticStatus::OK) {
    diag_toplevel_state.level = max_level_without_stale;
  } else if (max_level == diagnostic_msgs::msg::DiagnosticStatus::STALE) {
    diag_toplevel_state.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
  } else if (max_level < 0) {
    diag_toplevel_state.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
  } else {
    diag_toplevel_state.level = DiagnosticStatus::OK;
  }

  last_top_level_state_ = diag_toplevel_state.level;

  toplevel_state_pub_->publish(diag_toplevel_state);
}

rclcpp::Node::SharedPtr Aggregator::get_node() const
{
  RCLCPP_DEBUG(logger_, "get_node()");
  return this->n_;
}

}  // namespace diagnostic_aggregator
