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

/**< \author Kevin Watts */
/**< \author Arne Nordmann */

#include "diagnostic_aggregator/generic_analyzer.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/parameter.hpp"

PLUGINLIB_EXPORT_CLASS(diagnostic_aggregator::GenericAnalyzer, diagnostic_aggregator::Analyzer)

namespace diagnostic_aggregator
{
using std::string;
using std::vector;

GenericAnalyzer::GenericAnalyzer() {}

bool GenericAnalyzer::init(
  const std::string & path, const std::string & breadcrumb, const rclcpp::Node::SharedPtr n)
{
  path_ = path;
  breadcrumb_ = breadcrumb;
  nice_name_ = breadcrumb;
  RCLCPP_DEBUG(
    rclcpp::get_logger("GenericAnalyzer"), "GenericAnalyzer, breadcrumb: %s", breadcrumb_.c_str());

  std::map<std::string, rclcpp::Parameter> parameters;
  if (!n->get_parameters(breadcrumb_, parameters)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GenericAnalyzer"),
      "Couldn't retrieve parameters for generic analyzer at prefix '%s'.", breadcrumb_.c_str());
    return false;
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("GenericAnalyzer"), "Retrieved %zu parameter(s) for prefix '%s'.",
    parameters.size(), breadcrumb_.c_str());

  double timeout = 5.0;
  int num_items_expected = -1;
  bool discard_stale = false;

  for (const auto & param : parameters) {
    string pname = param.first;
    rclcpp::Parameter pvalue = param.second;

    if (pname.compare("path") == 0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("GenericAnalyzer"), "GenericAnalyzer '%s' found path: %s",
        nice_name_.c_str(), pvalue.value_to_string().c_str());
      nice_name_ = pvalue.as_string();
    } else if (pname.compare("find_and_remove_prefix") == 0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("GenericAnalyzer"),
        "GenericAnalyzer '%s' found find_and_remove_prefix: %s", nice_name_.c_str(),
        pvalue.value_to_string().c_str());
      vector<string> output = pvalue.as_string_array();
      chaff_ = output;
      startswith_ = output;
    } else if (pname.compare("remove_prefix") == 0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("GenericAnalyzer"), "GenericAnalyzer '%s' found remove_prefix: %s",
        nice_name_.c_str(), pvalue.value_to_string().c_str());
      chaff_ = pvalue.as_string_array();
    } else if (pname.compare("startswith") == 0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("GenericAnalyzer"), "GenericAnalyzer '%s' found startswith: %s",
        nice_name_.c_str(), pvalue.value_to_string().c_str());
      startswith_ = pvalue.as_string_array();
    } else if (pname.compare("contains") == 0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("GenericAnalyzer"), "GenericAnalyzer '%s' found contains: %s",
        nice_name_.c_str(), pvalue.value_to_string().c_str());
      contains_ = pvalue.as_string_array();
    } else if (pname.compare("expected") == 0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("GenericAnalyzer"), "GenericAnalyzer '%s' found expected: %s",
        nice_name_.c_str(), pvalue.value_to_string().c_str());
      for (auto exp : pvalue.as_string_array()) {
        auto item = std::make_shared<StatusItem>(exp);
        this->addItem(exp, item);
      }
    } else if (pname.compare("regex") == 0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("GenericAnalyzer"), "GenericAnalyzer '%s' found regex: %s",
        nice_name_.c_str(), pvalue.value_to_string().c_str());
      for (auto regex : pvalue.as_string_array()) {
        try {
          std::regex re(regex);
          regex_.push_back(re);
        } catch (std::regex_error & e) {
          RCLCPP_ERROR(
            rclcpp::get_logger("GenericAnalyzer"),
            "Attempted to make regex from %s. Caught exception, ignoring value. Exception: %s",
            regex.c_str(), e.what());
        }
      }
    } else if (pname.compare("timeout") == 0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("GenericAnalyzer"), "GenericAnalyzer '%s' found timeout: %s",
        nice_name_.c_str(), pvalue.value_to_string().c_str());
      timeout = pvalue.as_double();
    } else if (pname.compare("num_items") == 0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("GenericAnalyzer"), "GenericAnalyzer '%s' found num_items: %s",
        nice_name_.c_str(), pvalue.value_to_string().c_str());
      num_items_expected = static_cast<int>(pvalue.as_int());
    } else if (pname.compare("discard_stale") == 0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("GenericAnalyzer"), "GenericAnalyzer '%s' found discard_stale: %s",
        nice_name_.c_str(), pvalue.value_to_string().c_str());
      discard_stale = pvalue.as_bool();
    }
  }

  if (
    startswith_.size() == 0 && name_.size() == 0 && contains_.size() == 0 &&
    expected_.size() == 0 && regex_.size() == 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("generic_analyzer"),
      "GenericAnalyzer '%s' was not initialized with any way of checking diagnostics."
      "Name: %s, namespace: %s",
      nice_name_.c_str(), path.c_str(), n->get_namespace());
    return false;
  }

  // convert chaff_ to output name format. Fixes #17
  for (size_t i = 0; i < chaff_.size(); i++) {
    chaff_[i] = getOutputName(chaff_[i]);
  }

  string my_path;
  if (path == "/") {
    my_path = nice_name_;
  } else {
    my_path = path + "/" + nice_name_;
  }

  if (my_path.find("/") != 0) {
    my_path = "/" + my_path;
  }

  return GenericAnalyzerBase::init(path_, breadcrumb_, timeout, num_items_expected, discard_stale);
}

GenericAnalyzer::~GenericAnalyzer() {}

bool GenericAnalyzer::match(const string & name)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("GenericAnalyzer"), "Analyzer '%s' match %s", nice_name_.c_str(),
    name.c_str());

  std::cmatch what;
  for (unsigned int i = 0; i < regex_.size(); ++i) {
    if (std::regex_match(name.c_str(), what, regex_[i])) {
      RCLCPP_INFO(
        rclcpp::get_logger("GenericAnalyzer"), "Analyzer '%s' matches '%s' with regex.",
        nice_name_.c_str(), name.c_str());
      return true;
    }
  }

  for (unsigned int i = 0; i < expected_.size(); ++i) {
    if (name == expected_[i]) {
      RCLCPP_INFO(
        rclcpp::get_logger("GenericAnalyzer"), "Analyzer '%s' matches '%s'.", nice_name_.c_str(),
        name.c_str());
      return true;
    }
  }

  for (unsigned int i = 0; i < name_.size(); ++i) {
    if (name == name_[i]) {
      RCLCPP_INFO(
        rclcpp::get_logger("GenericAnalyzer"), "Analyzer '%s' matches '%s'.", nice_name_.c_str(),
        name.c_str());
      return true;
    }
  }

  for (unsigned int i = 0; i < startswith_.size(); ++i) {
    if (name.find(startswith_[i]) == 0) {
      RCLCPP_INFO(
        rclcpp::get_logger("GenericAnalyzer"), "Analyzer '%s' matches '%s'.", nice_name_.c_str(),
        name.c_str());
      return true;
    }
  }

  for (unsigned int i = 0; i < contains_.size(); ++i) {
    if (name.find(contains_[i]) != string::npos) {
      RCLCPP_INFO(
        rclcpp::get_logger("GenericAnalyzer"), "Analyzer '%s' matches '%s'.", nice_name_.c_str(),
        name.c_str());
      return true;
    }
  }

  return false;
}

vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> GenericAnalyzer::report()
{
  RCLCPP_DEBUG(rclcpp::get_logger("GenericAnalyzer"), "Analyzer '%s' report()", nice_name_.c_str());

  vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> processed =
    GenericAnalyzerBase::report();

  // Check and make sure our expected names haven't been removed ...
  vector<string> expected_names_missing;
  bool has_name = false;

  for (unsigned int i = 0; i < expected_.size(); ++i) {
    has_name = false;
    for (unsigned int j = 0; j < processed.size(); ++j) {
      size_t last_slash = processed[j]->name.rfind("/");
      string nice_name = processed[j]->name.substr(last_slash + 1);
      if (nice_name == expected_[i] || nice_name == getOutputName(expected_[i])) {
        has_name = true;
        break;
      }

      // Remove chaff, check names
      for (unsigned int k = 0; k < chaff_.size(); ++k) {
        if (nice_name == removeLeadingNameChaff(expected_[i], chaff_[k])) {
          has_name = true;
          break;
        }
      }
    }
    if (!has_name) {
      expected_names_missing.push_back(expected_[i]);
    }
  }

  // Check that all processed items aren't stale
  bool all_stale = true;
  for (unsigned int j = 0; j < processed.size(); ++j) {
    if (processed[j]->level != diagnostic_msgs::msg::DiagnosticStatus::STALE) {
      all_stale = false;
    }
  }

  // Add missing names to header ...
  for (unsigned int i = 0; i < expected_names_missing.size(); ++i) {
    std::shared_ptr<StatusItem> item(new StatusItem(expected_names_missing[i]));
    processed.push_back(item->toStatusMsg(path_, true));
  }

  for (unsigned int j = 0; j < processed.size(); ++j) {
    // Remove all leading name chaff
    for (unsigned int i = 0; i < chaff_.size(); ++i) {
      processed[j]->name = removeLeadingNameChaff(processed[j]->name, chaff_[i]);
    }

    // If we're missing any items, set the header status to error or stale
    if (expected_names_missing.size() > 0 && processed[j]->name == path_) {
      if (!all_stale) {
        processed[j]->level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        processed[j]->message = "Error";
      } else {
        processed[j]->level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
        processed[j]->message = "All Stale";
      }

      // Add all missing items to header item
      for (unsigned int k = 0; k < expected_names_missing.size(); ++k) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = expected_names_missing[k];
        kv.value = "Missing";
        processed[j]->values.push_back(kv);
      }
    }
  }

  return processed;
}

}  // namespace diagnostic_aggregator
