// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <map>
#include <string>
#include <vector>
#include <memory>
#include "diagnostic_aggregator/generic_analyzer.hpp"
using namespace std;
using namespace diagnostic_aggregator;
// namespace std {

PLUGINLIB_EXPORT_CLASS(diagnostic_aggregator::GenericAnalyzer,
  diagnostic_aggregator::Analyzer)

GenericAnalyzer::GenericAnalyzer() {}

bool GenericAnalyzer::init(
  const string base_path, const char * nsp,
  const rclcpp::Node::SharedPtr & n, const char * rnsp)
{
  auto context =
    rclcpp::contexts::default_context::get_global_default_context();
  const std::vector<std::string> arguments = {};
  const std::vector<rclcpp::Parameter> initial_values = {
    rclcpp::Parameter("base_path_ga", base_path),
  };
  const bool use_global_arguments = true;
  const bool use_intra_process = true;

  string gen_an_name = rnsp;
  gen_an_name.erase(gen_an_name.end() - 5, gen_an_name.end());

  gen_nh = n;
  string nice_name;
  auto parameters_client =
    std::make_shared<rclcpp::SyncParametersClient>(gen_nh, nsp);
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(gen_nh->get_logger(),
        "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(gen_nh->get_logger(),
      "service not available, waiting again...");
  }
  std::stringstream ss;
  std::stringstream ss1;
#if 0
  for (auto & parameter : parameters_client->get_parameters({"startswith"})) {
    ss << "\nParameter name: " << parameter.get_name();
    ss << "\nParameter value (" << parameter.get_type_name() <<
      "): " << parameter.value_to_string();
  }
#endif
  map<string, string> anl_param;

  auto parameters_and_prefixes =
    parameters_client->list_parameters({gen_an_name}, 10);

  ss << "\nParameter names:";
  for (auto & name : parameters_and_prefixes.names) {
    //  ss << "\n " << name;
    for (auto & parameter : parameters_client->get_parameters({name})) {
      ss1 << "\nParameter name: " << parameter.get_name();
      ss1 << "\nParameter value (" << parameter.get_type_name() <<
        "): " << parameter.value_to_string();
      anl_param[parameter.get_name()] = parameter.value_to_string();
    }
  }
  map<string, string>::iterator anl_it;
  anl_it = anl_param.find(gen_an_name + ".path");
  if (anl_it != anl_param.end()) {
    nice_name = anl_it->second;
  } else {
    return false;
  }
  anl_it = anl_param.find(gen_an_name + ".find_and_remove_prefix");
  if (anl_it != anl_param.end()) {
    string find_remove = anl_it->second;
    vector<string> output;
    if (getParamVals(find_remove, output)) {
      chaff_ = output;
      startswith_ = output;
    } else {
    }
  }

  anl_it = anl_param.find(gen_an_name + ".remove_prefix");
  if (anl_it != anl_param.end()) {
    string remove = anl_it->second;
    getParamVals(remove, chaff_);
  }

  anl_it = anl_param.find(gen_an_name + ".startswith");
  if (anl_it != anl_param.end()) {
    string startswith = anl_it->second;
    getParamVals(startswith, startswith_);
  }

  anl_it = anl_param.find(gen_an_name + ".name");
  if (anl_it != anl_param.end()) {
    string name = anl_it->second;
    getParamVals(name, name_);
  }

  anl_it = anl_param.find(gen_an_name + ".contains");
  if (anl_it != anl_param.end()) {
    string contains = anl_it->second;
    getParamVals(contains, contains_);
  }

  anl_it = anl_param.find(gen_an_name + ".expected");
  if (anl_it != anl_param.end()) {
    string expected = anl_it->second;
    // expected.erase(expected.begin() + 0);
    // expected.erase(expected.end() + 0) ;
    getParamVals(expected, expected_);
    for (unsigned int i = 0; i < expected_.size(); ++i) {
      std::shared_ptr<StatusItem> item(new StatusItem(expected_[i]));
      addItem(expected_[i], item);
    }
  }

  anl_it = anl_param.find(gen_an_name + ".regex");
  if (anl_it != anl_param.end()) {
    string regexes = anl_it->second;
    vector<string> regex_strs;
    getParamVals(regexes, regex_strs);

    for (unsigned int i = 0; i < regex_strs.size(); ++i) {
      try {
        std::regex re(regex_strs[i]);
        regex_.push_back(re);
      } catch (std::regex_error & e) {
        ROS_ERROR("Attempted to make regex from %s. Caught exception, ignoring "
          "value. Exception: %s",
          regex_strs[i].c_str(), e.what());
      }
    }
  }

  if (startswith_.size() == 0 && name_.size() == 0 && contains_.size() == 0 &&
    expected_.size() == 0 && regex_.size() == 0)
  {
    ROS_ERROR("GenericAnalyzer was not initialized with any way of checking "
      "diagnostics. Name: %s, namespace:",
      nice_name.c_str());
    return false;
  }

  // convert chaff_ to output name format. Fixes #17
  for (size_t i = 0; i < chaff_.size(); i++) {
    chaff_[i] = getOutputName(chaff_[i]);
  }

  double timeout;
  int num_items_expected;
  bool discard_stale;

  anl_it = anl_param.find(gen_an_name + ".timeout");
  if (anl_it != anl_param.end()) {
    timeout = stod(anl_it->second);
  } else {
    timeout = 5.0;
  }
  anl_it = anl_param.find(gen_an_name + ".num_items");
  if (anl_it != anl_param.end()) {
    num_items_expected = stod(anl_it->second);
  } else {
    num_items_expected = -1;
  }

  anl_it = anl_param.find(gen_an_name + ".discard_stale");
  if (anl_it != anl_param.end()) {
    if (anl_it->second == "true") {
      discard_stale = true;
    }
  } else {
    discard_stale = false;
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
  return GenericAnalyzerBase::init_v(my_path, nice_name, timeout,
           num_items_expected, discard_stale);
}

GenericAnalyzer::~GenericAnalyzer() {}

bool GenericAnalyzer::match(const string name)
{
  std::cmatch what;
  for (unsigned int i = 0; i < regex_.size(); ++i) {
    if (std::regex_match(name.c_str(), what, regex_[i])) {
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

vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>>
GenericAnalyzer::report()
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
      if (nice_name == expected_[i] ||
        nice_name == getOutputName(expected_[i]))
      {
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
      processed[j]->name =
        removeLeadingNameChaff(processed[j]->name, chaff_[i]);
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
//}
