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

/**
 * @todo(anordman): make aggregator a lifecycle node.
 */
Aggregator::Aggregator()
: n_(std::make_shared<rclcpp::Node>("analyzers",
    "",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))),
  logger_(rclcpp::get_logger("Aggregator")),
  pub_rate_(1.0),
  clock_(new rclcpp::Clock()),
  base_path_("/")
{
  RCLCPP_DEBUG(logger_, "constructor");
  bool other_as_errors = false;

  std::map<std::string, rclcpp::Parameter> parameters;
  if (!n_->get_parameters("", parameters)) {
    RCLCPP_ERROR(logger_, "Couldn't retrieve parameters.");
  }
  RCLCPP_DEBUG(logger_, "Retrieved %d parameter(s).", parameters.size());

  for (const auto & param : parameters) {
    if (param.first.compare("pub_rate") == 0) {
      pub_rate_ = param.second.as_double();
    } else if (param.first.compare("path") == 0) {
      base_path_.append(param.second.as_string());
    } else if (param.first.compare("other_as_errors") == 0) {
      other_as_errors = param.second.as_bool();
    }
  }
  RCLCPP_DEBUG(logger_, "Aggregator publication rate configured to: %f", pub_rate_);
  RCLCPP_DEBUG(logger_, "Aggregator base path configured to: %s", base_path_.c_str());
  RCLCPP_DEBUG(logger_, "Aggregator other_as_errors configured to: %s",
    (other_as_errors ? "true" : "false"));

  analyzer_group_ = std::make_unique<AnalyzerGroup>();
  if (!analyzer_group_->init(base_path_, "", n_)) {
    RCLCPP_ERROR(logger_, "Analyzer group for diagnostic aggregator failed to initialize!");
  }

  // Last analyzer handles remaining data
  other_analyzer_ = std::make_unique<OtherAnalyzer>(other_as_errors);
  other_analyzer_->init(base_path_);  // This always returns true
  
  // @todo(anordman): This cout somehow is necessary to continue screen output for testing
  std::cout << "Aggregator created, starting analysis..." << std::endl;

  add_srv_ = n_->create_service<diagnostic_msgs::srv::AddDiagnostics>(
    "/diagnostics_agg/add_diagnostics",
    std::bind(&Aggregator::addDiagnostics, this, _1, _2, _3));
  diag_sub_ = n_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",
      rclcpp::SystemDefaultsQoS(), std::bind(
        &Aggregator::diagCallback, this,
        _1));
  agg_pub_ = n_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics_agg", 1);
  toplevel_state_pub_ = n_->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    "/diagnostics_toplevel_state", 1);
}

void Aggregator::checkTimestamp(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr diag_msg)
{
  RCLCPP_DEBUG(logger_, "checkTimestamp()");
  if (diag_msg->header.stamp.sec != 0) {
    return;
  }

  std::string stamp_warn = "No timestamp set for diagnostic message. Message names: ";
  std::vector<diagnostic_msgs::msg::DiagnosticStatus>::const_iterator it;
  for (it = diag_msg->status.begin(); it != diag_msg->status.end(); ++it) {
    if (it != diag_msg->status.begin()) {
      stamp_warn += ", ";
    }
    stamp_warn += it->name;
  }

  if (!ros_warnings_.count(stamp_warn)) {
    RCLCPP_WARN(logger_, "%s", stamp_warn.c_str());
    ros_warnings_.insert(stamp_warn);
  }
}

void Aggregator::diagCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr diag_msg)
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

void Aggregator::bondBroken(std::string bond_id, std::shared_ptr<Analyzer> analyzer)
{
  RCLCPP_DEBUG(logger_, "bondBroken()");
  std::lock_guard<std::mutex> lock(mutex_);  // Possibility of multiple bonds breaking at once
  RCLCPP_WARN(logger_, "Bond for namespace %s was broken", bond_id.c_str());
  std::vector<std::shared_ptr<bond::Bond>>::iterator elem;
  elem = std::find_if(bonds_.begin(), bonds_.end(), BondIDMatch(bond_id));
  if (elem == bonds_.end()) {
    RCLCPP_WARN(logger_, "Broken bond tried to erase a bond which didn't exist.");
  } else {
    bonds_.erase(elem);
  }
  if (!analyzer_group_->removeAnalyzer(analyzer)) {
    RCLCPP_WARN(logger_, "Broken bond tried to remove an analyzer which didn't exist.");
  }

  analyzer_group_->resetMatches();
}

void Aggregator::bondFormed(std::shared_ptr<Analyzer> group)
{
  RCLCPP_DEBUG(logger_, "bondFormed()");
  std::lock_guard<std::mutex> lock(mutex_);
  analyzer_group_->addAnalyzer(group);
  analyzer_group_->resetMatches();
}

bool Aggregator::addDiagnostics(
  const std::shared_ptr<rmw_request_id_t> header,
  const std::shared_ptr<diagnostic_msgs::srv::AddDiagnostics::Request> req,
  std::shared_ptr<diagnostic_msgs::srv::AddDiagnostics::Response> res)
{
  (void) header;

  RCLCPP_DEBUG(logger_, "Got load request for namespace %s", req->load_namespace.c_str());
  // Don't currently support relative or private namespace definitions
  if (req->load_namespace[0] != '/') {
    res->message =
      R"(Requested load from non-global namespace.
      Private and relative namespaces are not supported.)";
    res->success = false;
    return true;
  }

  std::shared_ptr<Analyzer> group = std::make_shared<AnalyzerGroup>();
  { // lock here ensures that bonds from the same namespace aren't added twice.
    // Without it, possibility of two simultaneous calls adding two objects.
    std::lock_guard<std::mutex> lock(mutex_);
    // rebuff attempts to add things from the same namespace twice
    if (std::find_if(bonds_.begin(), bonds_.end(),
      BondIDMatch(req->load_namespace)) != bonds_.end())
    {
      res->message = "Requested load from namespace " + req->load_namespace +
        " which is already in use";
      res->success = false;
      return true;
    }

    // Use a different topic for each bond to help control the message queue
    // length. Bond has a fixed size subscriber queue, so we can easily miss
    // bond heartbeats if there are too many bonds on the same topic.
    std::shared_ptr<bond::Bond> req_bond = std::make_shared<bond::Bond>(
      "/diagnostics_agg/bond" + req->load_namespace, req->load_namespace, n_,
      std::function<void(void)>(std::bind(&Aggregator::bondBroken, this, req->load_namespace,
      group)),
      std::function<void(void)>(std::bind(&Aggregator::bondFormed, this, group))
    );
    req_bond->start();

    bonds_.push_back(req_bond);  // bond formed, keep track of it
  }

  if (group->init(base_path_, "", std::make_shared<rclcpp::Node>(req->load_namespace))) {
    res->message = "Successfully initialised AnalyzerGroup. Waiting for bond to form.";
    res->success = true;
    return true;
  } else {
    res->message = "Failed to initialise AnalyzerGroup.";
    res->success = false;
    return true;
  }
}

void Aggregator::publishData()
{
  RCLCPP_DEBUG(logger_, "publishData()");
  diagnostic_msgs::msg::DiagnosticArray diag_array;

  diagnostic_msgs::msg::DiagnosticStatus diag_toplevel_state;
  diag_toplevel_state.name = "toplevel_state";
  diag_toplevel_state.level = -1;
  int min_level = 255;

  std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> processed;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    processed = analyzer_group_->report();
  }
  for (const auto & msg : processed) {
    diag_array.status.push_back(*msg);

    if (msg->level > diag_toplevel_state.level) {
      diag_toplevel_state.level = msg->level;
    }
    if (msg->level < min_level) {
      min_level = msg->level;
    }
  }

  std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> processed_other =
    other_analyzer_->report();
  for (const auto & msg : processed_other) {
    diag_array.status.push_back(*msg);

    if (msg->level > diag_toplevel_state.level) {
      diag_toplevel_state.level = msg->level;
    }
    if (msg->level < min_level) {
      min_level = msg->level;
    }
  }

  diag_array.header.stamp = clock_->now();

  agg_pub_->publish(diag_array);

  // Top level is error if we have stale items, unless all stale
  if (diag_toplevel_state.level > 2 && min_level <= 2) {
    diag_toplevel_state.level = 2;
  }

  toplevel_state_pub_->publish(diag_toplevel_state);
}

rclcpp::Node::SharedPtr
Aggregator::get_node() const
{
  RCLCPP_DEBUG(logger_, "get_node()");
  return this->n_;
}

}  // namespace diagnostic_aggregator
