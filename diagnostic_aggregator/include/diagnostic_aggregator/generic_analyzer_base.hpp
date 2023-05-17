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

/**!< \author Kevin Watts */
/**!< \author Arne Nordmann */

#ifndef DIAGNOSTIC_AGGREGATOR__GENERIC_ANALYZER_BASE_HPP_
#define DIAGNOSTIC_AGGREGATOR__GENERIC_ANALYZER_BASE_HPP_

#include <algorithm>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "diagnostic_aggregator/analyzer.hpp"
#include "diagnostic_aggregator/status_item.hpp"
#include "diagnostic_aggregator/visibility_control.hpp"

#include "diagnostic_msgs/msg/diagnostic_status.h"
#include "diagnostic_msgs/msg/key_value.h"

#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/rclcpp.hpp"

namespace diagnostic_aggregator
{
/*!
 *\brief GenericAnalyzerBase is the base class for GenericAnalyzer and OtherAnalyzer
 *
 * GenericAnalyzerBase contains the getPath(), getName(), analyze() and report() functions of
 * the Generic and Other Analyzers. It is a virtual class, and cannot be instantiated or loaded
 * as a plugin. Subclasses are responsible for implementing the init() and match() functions.
 *
 * The GenericAnalyzerBase holds the state of the analyzer, and tracks if items are stale, and
 * if the user has the correct number of items.
 */
class GenericAnalyzerBase : public Analyzer
{
public:
  GenericAnalyzerBase()
  : nice_name_(""),
    path_(""),
    timeout_(-1.0),
    num_items_expected_(-1),
    discard_stale_(false),
    has_initialized_(false),
    has_warned_(false)
  {
  }

  virtual ~GenericAnalyzerBase()
  {
    items_.clear();
  }

  /*
   *\brief Cannot be initialized from (string, NodeHandle) like defined Analyzers
   */
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  bool init(
    const std::string & base_path, const std::string & breadcrumb,
    const rclcpp::Node::SharedPtr node) = 0;

  /*
   *\brief Must be initialized with path, and a "nice name"
   *
   * Must be initialized in order to prepend the path to all outgoing status messages.
   */
  bool init(
    const std::string & path, const std::string & breadcrumb, double timeout = -1.0,
    int num_items_expected = -1, bool discard_stale = false)
  {
    num_items_expected_ = num_items_expected;
    timeout_ = timeout;
    path_ = path + "/" + nice_name_;
    discard_stale_ = discard_stale;
    breadcrumb_ = breadcrumb;

    if (discard_stale_ && timeout <= 0) {
      RCLCPP_WARN(
        rclcpp::get_logger("generic_analyzer_base"),
        "Cannot discard stale items if no timeout specified. No items will be discarded");
      discard_stale_ = false;
    }

    has_initialized_ = true;

    RCLCPP_INFO(
      rclcpp::get_logger("GenericAnalyzerBase"),
      "Initialized analyzer '%s' with path '%s' and breadcrumb '%s'.", nice_name_.c_str(),
      path_.c_str(), breadcrumb_.c_str());
    return true;
  }

  /*!
   *\brief Update state with new StatusItem
   */
  virtual bool analyze(const std::shared_ptr<StatusItem> item)
  {
    RCLCPP_DEBUG(
      rclcpp::get_logger("GenericAnalyzerBase"), "Analyzer '%s' analyze, item %s: %s",
      nice_name_.c_str(), item->getName().c_str(), item->getMessage().c_str());

    if (!has_initialized_ && !has_warned_) {
      has_warned_ = true;
      RCLCPP_WARN(
        rclcpp::get_logger(
          "generic_analyzer_base"),
        R"(GenericAnalyzerBase is asked to analyze diagnostics without being initialized.
        init() must be called in order to correctly use this class.)");
    }

    if (!has_initialized_) {
      return false;
    }

    items_[item->getName()] = item;

    return has_initialized_;
  }

  /*!
   *\brief Reports current state, returns vector of formatted status messages
   *
   *\return Vector of DiagnosticStatus messages. They must have the correct prefix for all names.
   */
  virtual std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> report()
  {
    RCLCPP_DEBUG(
      rclcpp::get_logger("GenericAnalyzerBase"), "Analyzer '%s' report()", nice_name_.c_str());

    if (!has_initialized_ && !has_warned_) {
      has_warned_ = true;
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "generic_analyzer_base"),
        R"("GenericAnalyzerBase is asked to report diagnostics without being initialized.
        init() must be called in order to correctly use this class.)");
    }
    if (!has_initialized_) {
      std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> vec;
      return vec;
    }

    auto header_status = std::make_shared<diagnostic_msgs::msg::DiagnosticStatus>();
    header_status->name = path_;
    header_status->level = 0;
    header_status->message = "OK";

    std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> processed;
    processed.push_back(header_status);

    bool all_stale = true;

    auto it = items_.begin();
    while (it != items_.end()) {
      auto name = it->first;
      auto item = it->second;

      bool stale = false;
      if (timeout_ > 0) {
        stale = (clock_->now() - item->getLastUpdateTime()).seconds() > timeout_;
      }

      // Erase item if its stale and we're discarding items
      if (discard_stale_ && stale) {
        items_.erase(it++);
        continue;
      }

      int8_t level = item->getLevel();
      header_status->level = std::max(static_cast<int8_t>(header_status->level), level);

      diagnostic_msgs::msg::KeyValue kv;
      kv.key = name;
      kv.value = item->getMessage();

      header_status->values.push_back(kv);

      all_stale = all_stale && ((level == diagnostic_msgs::msg::DiagnosticStatus::STALE) || stale);

      processed.push_back(item->toStatusMsg(path_, stale));

      if (stale) {
        header_status->level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      }

      ++it;
    }

    // Header is not stale unless all subs are
    if (all_stale) {
      header_status->level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    } else if (header_status->level == diagnostic_msgs::msg::DiagnosticStatus::STALE) {
      header_status->level = 2;
    }

    header_status->message = valToMsg(header_status->level);

    // If we expect a given number of items, check that we have this number
    if (num_items_expected_ == 0 && items_.empty()) {
      header_status->level = 0;
      header_status->message = "OK";
    } else if (  // NOLINT
      num_items_expected_ > 0 &&
      static_cast<int8_t>(items_.size()) != num_items_expected_)
    {  // NOLINT
      int8_t lvl = 2;
      header_status->level = std::max(lvl, static_cast<int8_t>(header_status->level));

      std::stringstream expec, item;
      expec << num_items_expected_;
      item << items_.size();

      if (!items_.empty()) {
        header_status->message = "Expected " + expec.str() + ", found " + item.str();
      } else {
        header_status->message = "No items found, expected " + expec.str();
      }
    }

    return processed;
  }

  /*!
   *\brief Match function isn't implemented by GenericAnalyzerBase
   */
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  virtual bool match(const std::string & name) = 0;

  /*!
   *\brief Returns full prefix (ex: "/Robot/Power System")
   */
  virtual std::string getPath() const {return path_;}

  /*!
   *\brief Returns nice name (ex: "Power System")
   */
  virtual std::string getName() const {return nice_name_;}

protected:
  /// Nice analyzer name
  std::string nice_name_;
  /// Nice analyzer path (up to parent)
  std::string path_;
  /// Dotted parameter path in yaml
  std::string breadcrumb_;

  double timeout_;
  int num_items_expected_;

  /*!
   *\brief Subclasses can add items to analyze
   */
  void addItem(std::string name, std::shared_ptr<StatusItem> item)
  {
    items_[name] = item;
  }

private:
  /*!
   *\brief Stores items by name. State of analyzer
   */
  std::map<std::string, std::shared_ptr<StatusItem>> items_;

  bool discard_stale_, has_initialized_, has_warned_;
};

}  // namespace diagnostic_aggregator

#endif  // DIAGNOSTIC_AGGREGATOR__GENERIC_ANALYZER_BASE_HPP_
