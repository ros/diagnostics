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
: n_(std::make_shared<rclcpp::Node>(
      "analyzers", "",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))),
  logger_(rclcpp::get_logger("Aggregator")),
  pub_rate_(1.0),
  history_depth_(1000),
  clock_(n_->get_clock()),
  base_path_("")
{
  RCLCPP_DEBUG(logger_, "constructor");
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
    }
  }
  RCLCPP_DEBUG(logger_, "Aggregator publication rate configured to: %f", pub_rate_);
  RCLCPP_DEBUG(logger_, "Aggregator base path configured to: %s", base_path_.c_str());
  RCLCPP_DEBUG(
    logger_, "Aggregator other_as_errors configured to: %s", (other_as_errors ? "true" : "false"));

  analyzer_group_ = std::make_unique<AnalyzerGroup>();
  if (!analyzer_group_->init(base_path_, "", n_)) {
    RCLCPP_ERROR(logger_, "Analyzer group for diagnostic aggregator failed to initialize!");
  }

  // Last analyzer handles remaining data
  other_analyzer_ = std::make_unique<OtherAnalyzer>(other_as_errors);
  other_analyzer_->init(base_path_);  // This always returns true

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
    }
  }
}

Aggregator::~Aggregator()
{
  RCLCPP_DEBUG(logger_, "destructor");
}

void Aggregator::publishData()
{
  RCLCPP_DEBUG(logger_, "publishData()");
  DiagnosticArray diag_array;
  DiagnosticStatus diag_toplevel_state;
  diag_toplevel_state.name = "toplevel_state";
  diag_toplevel_state.level = DiagnosticStatus::STALE;
  int max_level = -1;
  int min_level = 255;

  std::vector<std::shared_ptr<DiagnosticStatus>> processed;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    processed = analyzer_group_->report();
  }
  for (const auto & msg : processed) {
    diag_array.status.push_back(*msg);

    if (msg->level > max_level) {
      max_level = msg->level;
    }
    if (msg->level < min_level) {
      min_level = msg->level;
    }
  }

  std::vector<std::shared_ptr<DiagnosticStatus>> processed_other =
    other_analyzer_->report();
  for (const auto & msg : processed_other) {
    diag_array.status.push_back(*msg);

    if (msg->level > max_level) {
      max_level = msg->level;
    }
    if (msg->level < min_level) {
      min_level = msg->level;
    }
  }

  diag_array.header.stamp = clock_->now();
  agg_pub_->publish(diag_array);

  diag_toplevel_state.level = max_level;
  if (max_level < 0 ||
    (max_level > DiagnosticStatus::ERROR && min_level <= DiagnosticStatus::ERROR))
  {
    // Top level is error if we got no diagnostic level or
    // have stale items but not all are stale
    diag_toplevel_state.level = DiagnosticStatus::ERROR;
  }
  toplevel_state_pub_->publish(diag_toplevel_state);
}

rclcpp::Node::SharedPtr Aggregator::get_node() const
{
  RCLCPP_DEBUG(logger_, "get_node()");
  return this->n_;
}

}  // namespace diagnostic_aggregator
