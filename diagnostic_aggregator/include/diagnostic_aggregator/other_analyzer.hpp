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

/*!
 * \author Kevin Watts
 */

#ifndef DIAGNOSTIC_AGGREGATOR__OTHER_ANALYZER_HPP_
#define DIAGNOSTIC_AGGREGATOR__OTHER_ANALYZER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "diagnostic_aggregator/generic_analyzer_base.hpp"
#include "diagnostic_aggregator/visibility_control.hpp"

#include "rclcpp/rclcpp.hpp"

namespace diagnostic_aggregator
{
/*
 *\brief OtherAnalyzer analyzes any messages that haven't been analyzed by other Analyzers
 *
 * OtherAnalyzer is not loaded as a plugin. It is created by the Aggregator, and called
 * seperately. The aggregator will call analyze() on any message not handled by other
 * analyzers.
 *
 * Stale items will be discarded after 5 seconds of no updates.
 *
 * OtherAnalyzer is designed to be used internally by the Aggregator only.
 *
 */
class OtherAnalyzer : public GenericAnalyzerBase
{
public:
  /*!
   *\brief Default constructor. OtherAnalyzer isn't loaded by pluginlib
   */
  explicit OtherAnalyzer(bool other_as_errors = false)
  : other_as_errors_(other_as_errors)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("OtherAnalyzer"), "constructor");
  }

  virtual ~OtherAnalyzer()
  {
    RCLCPP_DEBUG(rclcpp::get_logger("OtherAnalyzer"), "destructor");
  }

  /*
   *\brief Initialized with the base path only.
   *
   *\param path Base path of Aggregator
   *\param breadcrumb Prefix for parameter getter.
   */
  bool init(const std::string & path, const std::string & breadcrumb = "")
  {
    (void)breadcrumb;

    nice_name_ = "Other";
    path_ = path;
    return GenericAnalyzerBase::init(path_, "", 5.0, -1, true);
  }

  /*
   *\brief OtherAnalyzer cannot be initialized with a NodeHandle
   *
   *\return False, since NodeHandle initialization isn't valid
   */
  bool init(
    const std::string & base_path, const std::string & breadcrumb,
    const rclcpp::Node::SharedPtr node)
  {
    (void)base_path;
    (void)breadcrumb;
    (void)node;

    RCLCPP_ERROR(
      rclcpp::get_logger(
        "generic_analyzer_base"),
      R"(OtherAnalyzer was attempted to initialize with a NodeHandle.
      This analyzer cannot be used as a plugin.)");
    return false;
  }

  /*
   *\brief match() isn't called by aggregator for OtherAnalyzer
   *
   *\return True, since match() will never by called by Aggregator
   */
  bool match(const std::string & name)
  {
    (void)name;

    return true;
  }

  /*
   *\brief Reports diagnostics, but doesn't report anything if it doesn't have data
   *
   */
  std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> report()
  {
    std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> processed =
      GenericAnalyzerBase::report();

    // We don't report anything if there's no "Other" items
    if (processed.size() == 1) {
      processed.clear();
    } else if (other_as_errors_ && processed.size() > 1) {
      // "Other" items are considered an error.
      std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>>::iterator it =
        processed.begin();
      for (; it != processed.end(); ++it) {
        if ((*it)->name == path_) {
          (*it)->level = 2;
          (*it)->message = "Unanalyzed items found in \"Other\"";
          break;
        }
      }
    }

    return processed;
  }

private:
  bool other_as_errors_;
};

}  // namespace diagnostic_aggregator

#endif  // DIAGNOSTIC_AGGREGATOR__OTHER_ANALYZER_HPP_
