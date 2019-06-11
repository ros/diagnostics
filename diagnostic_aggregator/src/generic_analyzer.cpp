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

#include <rclcpp/parameter.hpp>

#include "diagnostic_aggregator/generic_analyzer.hpp"

using namespace diagnostic_aggregator;
using namespace std;

PLUGINLIB_EXPORT_CLASS(diagnostic_aggregator::GenericAnalyzer,
  diagnostic_aggregator::Analyzer)


GenericAnalyzer::GenericAnalyzer() {}

bool GenericAnalyzer::init(const string base_path, const rclcpp::Node::SharedPtr n)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("generic_analyzer"),
    "GenericAnalyzer(), base_path: %s, namespace: %s",
    base_path.c_str(),
    n->get_namespace());
  
  std::map<std::string, rclcpp::Parameter> parameters;
  if (!n->get_parameters(base_path, parameters)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("generic_analyzer"),
      "Couldn't retrieve parameters for generic analyzer '%s', namespace '%s'.",
      base_path.c_str(), n->get_namespace());
      return false;
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("generic_analyzer"),
    "Retrieved %d parameter(s) for generic analyzer '%s.",
    parameters.size(), base_path.c_str());
  
  string nice_name = base_path; //@todo(anordman): check, what is this about?
  double timeout;
  int num_items_expected;
  bool discard_stale;
  
  for (auto param : parameters) {
    string pname = param.first;
    rclcpp::Parameter pvalue = param.second;    
    RCLCPP_DEBUG(rclcpp::get_logger("generic_analyzer"), "param: %s : %s", pname.c_str(), pvalue.value_to_string().c_str());
    
    if (pname.compare("name") == 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("generic_analyzer"), "GenericAnalyzer name: %s", pvalue.value_to_string().c_str());
      name_ = pvalue.as_string_array();
    } else if (pname.compare("find_and_remove_prefix") == 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("generic_analyzer"), "GenericAnalyzer find_and_remove_prefix: %s", pvalue.value_to_string().c_str());
      vector<string> output = { pname };
      chaff_ = output;
      startswith_ = output;
    } else if (pname.compare("remove_prefix") == 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("generic_analyzer"), "GenericAnalyzer remove_prefix: %s", pvalue.value_to_string().c_str());
      chaff_ = pvalue.as_string_array();
    } else if (pname.compare("startswith") == 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("generic_analyzer"), "GenericAnalyzer startswith: %s", pvalue.value_to_string().c_str());
      startswith_ = pvalue.as_string_array();
    } else if (pname.compare("contains") == 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("generic_analyzer"), "GenericAnalyzer contains: %s", pvalue.value_to_string().c_str());
      contains_ = pvalue.as_string_array();
    } else if (pname.compare("expected") == 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("generic_analyzer"), "GenericAnalyzer expected: %s", pvalue.value_to_string().c_str());
      for (auto exp : pvalue.as_string_array()) {
        auto item = std::make_shared<StatusItem>(exp);
        this->addItem(exp, item);
      }
    } else if (pname.compare("regex") == 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("generic_analyzer"), "GenericAnalyzer regex: %s", pvalue.value_to_string().c_str());
      for (auto regex : pvalue.as_string_array()) {
        try {
          boost::regex re(regex);
          regex_.push_back(re);
        } catch (boost::regex_error & e) {
          RCLCPP_ERROR(rclcpp::get_logger("generic_analyzer"),
            "Attempted to make regex from %s. Caught exception, ignoring value. Exception: %s",
            regex.c_str(), e.what());
        }
      }
    } else if (pname.compare("timeout") == 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("generic_analyzer"), "GenericAnalyzer timeout: %s", pvalue.value_to_string().c_str());
      timeout = pvalue.as_double();
    } else if (pname.compare("num_items") == 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("generic_analyzer"), "GenericAnalyzer num_items: %s", pvalue.value_to_string().c_str());
      num_items_expected = pvalue.as_int();
    } else if (pname.compare("discard_stale") == 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("generic_analyzer"), "GenericAnalyzer discard_stale: %s", pvalue.value_to_string().c_str());
      discard_stale = pvalue.as_bool();
    }
  }

  if (startswith_.size() == 0 && name_.size() == 0 &&
    contains_.size() == 0 && expected_.size() == 0 && regex_.size() == 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(
        "generic_analyzer"),
      "GenericAnalyzer was not initialized with any way of checking diagnostics. Name: %s, namespace: %s",
      base_path.c_str(), n->get_namespace());
    return false;
  }
  
  // convert chaff_ to output name format. Fixes #17
  for (size_t i = 0; i < chaff_.size(); i++) {
    chaff_[i] = getOutputName(chaff_[i]);
  }
  
  string my_path;
  if (base_path == "/") {
    my_path = nice_name;
  } else {
    my_path = base_path + "/" + nice_name;
  }

  if (my_path.find("/") != 0) {
    my_path = "/" + my_path;
  }

  return GenericAnalyzerBase::init(my_path, nice_name, timeout, num_items_expected, discard_stale);
}

GenericAnalyzer::~GenericAnalyzer() {}


bool GenericAnalyzer::match(const string name)
{
  boost::cmatch what;
  for (unsigned int i = 0; i < regex_.size(); ++i) {
    if (boost::regex_match(name.c_str(), what, regex_[i])) {
      return true;
    }
  }

  for (unsigned int i = 0; i < expected_.size(); ++i) {
    if (name == expected_[i]) {
      return true;
    }
  }

  for (unsigned int i = 0; i < name_.size(); ++i) {
    if (name == name_[i]) {
      return true;
    }
  }

  for (unsigned int i = 0; i < startswith_.size(); ++i) {
    if (name.find(startswith_[i]) == 0) {
      return true;
    }
  }

  for (unsigned int i = 0; i < contains_.size(); ++i) {
    if (name.find(contains_[i]) != string::npos) {
      return true;
    }
  }

  return false;
}

vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> GenericAnalyzer::report()
{
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
    if (processed[j]->level != 3) {
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
        processed[j]->level = 2;
        processed[j]->message = "Error";
      } else {
        processed[j]->level = 3;
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
