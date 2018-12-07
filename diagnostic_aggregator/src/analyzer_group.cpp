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

#include <iostream>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_aggregator/analyzer_group.hpp"

using namespace std;
using namespace diagnostic_aggregator;
PLUGINLIB_EXPORT_CLASS(diagnostic_aggregator::AnalyzerGroup,
  diagnostic_aggregator::Analyzer)

// namespace std {
// namespace diagnostic_aggregator {
AnalyzerGroup::AnalyzerGroup()
: path_(""), nice_name_(""),
  analyzer_loader_("diagnostic_aggregator",
    "diagnostic_aggregator::Analyzer") {}

bool AnalyzerGroup::init(
  const string base_path, const char * nsp,
  const rclcpp::Node::SharedPtr & nh, const char * rnsp)
{
  auto context =
    rclcpp::contexts::default_context::get_global_default_context();
  const std::vector<std::string> arguments = {};
  const std::vector<rclcpp::Parameter> initial_values = {
    rclcpp::Parameter("an_base_path", ""),
    rclcpp::Parameter("an_pub_rate", 1.0),
  };
  const bool use_global_arguments = true;
  const bool use_intra_process = true;

  if (base_path.size() > 0 && base_path != "/") {
    path_ = base_path + "/" + nice_name_;
  } else {
    path_ = nice_name_;
  }

  if (path_.find("/") != 0) {
    path_ = "/" + path_;
  }

  string anz_name;
  string an_name = nsp;

  analyzers_nh = nh;

  rclcpp::SyncParametersClient::SharedPtr parameters_client_analyzer =
    std::make_shared<rclcpp::SyncParametersClient>(analyzers_nh, nsp);
  while (!parameters_client_analyzer->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(analyzers_nh->get_logger(),
        "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(analyzers_nh->get_logger(),
      "service not available, waiting again...");
  }

  RCLCPP_INFO(analyzers_nh->get_logger(), "service is  available Now ");
  std::stringstream ss;
  std::stringstream ss1;

  bool init_ok = true;

  map<string, string> anl_param;
  string params_anz, r_nsp;
  if (rnsp != NULL) {
    r_nsp = rnsp;
    r_nsp.erase(r_nsp.end() - 5, r_nsp.end());
    params_anz = r_nsp + ".analyzers_params";
  } else {
    params_anz = "analyzers_params";
  }

  auto parameters_and_prefixes =
    parameters_client_analyzer->list_parameters({params_anz}, 10);

  ss << "\nParameter names:";
  for (auto & name : parameters_and_prefixes.names) {
    //  ss << "\n " << name;
    for (auto & parameter : parameters_client_analyzer->get_parameters({name})) {
      ss1 << "\nParameter name: " << parameter.get_name();
      ss1 << "\nParameter value (" << parameter.get_type_name() <<
        "): " << parameter.value_to_string();
      anl_param[parameter.get_name()] = parameter.value_to_string();
    }
  }

  RCLCPP_INFO(analyzers_nh->get_logger(), ss1.str().c_str());

  for (map<string, string>::iterator anl_it = anl_param.begin();
    anl_it != anl_param.end(); ++anl_it)
  {
    string analyzer_name = anl_it->first;
    string ns = anl_it->second;
    cout << analyzer_name << "::" << ns << endl;
    std::shared_ptr<Analyzer> analyzer;
    string an_type = anl_it->second;
    if (std::string::npos != analyzer_name.find("type")) {
      cout << "===>> " << analyzer_name << endl;
      string params_anz_ = params_anz + ".";
      string p_name = analyzer_name;
      string::size_type i = p_name.find(params_anz_);
      if (i != std::string::npos) {
        p_name.erase(i, params_anz_.length());
      }
      if (std::string::npos != p_name.find("analyzers_params")) {
        continue;
      }
      try {
        // Look for non-fully qualified class name for Analyzer type
        if (!analyzer_loader_.isClassAvailable(an_type)) {
          bool have_class = false;
          vector<string> classes = analyzer_loader_.getDeclaredClasses();
          for (unsigned int i = 0; i < classes.size(); ++i) {
            if (an_type == analyzer_loader_.getName(classes[i])) {
              // if we've found a match... we'll get the fully qualified name
              // and break out of the loop
              ROS_WARN("Analyzer specification should now include the package "
                "name. You are using a deprecated API. Please switch "
                "from %s to %s in your Analyzer specification.",
                an_type.c_str(), classes[i].c_str());
              an_type = classes[i];
              have_class = true;
              break;
            }
            cout << "available clase is " << classes[i] << endl;
          }
          if (!have_class) {
            ROS_ERROR("Unable to find Analyzer class %s. Check that Analyzer "
              "is fully declared.",
              an_type.c_str());
            continue;
          }
        }

        analyzer = analyzer_loader_.createSharedInstance(an_type);
      } catch (pluginlib::LibraryLoadException & e) {
        ROS_ERROR("Failed to load analyzer %s, type %s. Caught exception. %s",
          ns.c_str(), an_type.c_str(), e.what());
        std::shared_ptr<StatusItem> item(
          new StatusItem(ns, "Pluginlib exception loading analyzer"));
        aux_items_.push_back(item);
        init_ok = false;
        continue;
      }

    } else {
      continue;
    }
    if (!analyzer) {
      std::shared_ptr<StatusItem> item(
        new StatusItem(ns, "Pluginlib return NULL Analyzer for " + an_type));
      aux_items_.push_back(item);
      init_ok = false;
      continue;
    }

    // if (!analyzer->init(path_, analyzer_name.c_str(),nh,rnsp))
    if (!analyzer->init(path_, nsp, nh, analyzer_name.c_str())) {
      // analyzers_nh.getNamespace().c_str(), an_type.c_str());
      std::shared_ptr<StatusItem> item(
        new StatusItem(ns, "Analyzer init failed"));
      aux_items_.push_back(item);
      init_ok = false;
      continue;
    }
    analyzers_.push_back(analyzer);
  }
  if (analyzers_.size() == 0) {
    init_ok = false;
  } else {
  }
  return init_ok;
}

AnalyzerGroup::~AnalyzerGroup() {analyzers_.clear();}

bool AnalyzerGroup::addAnalyzer(std::shared_ptr<Analyzer> & analyzer)
{
  analyzers_.push_back(analyzer);
  return true;
}

bool AnalyzerGroup::removeAnalyzer(std::shared_ptr<Analyzer> & analyzer)
{
  vector<std::shared_ptr<Analyzer>>::iterator it =
    find(analyzers_.begin(), analyzers_.end(), analyzer);
  if (it != analyzers_.end()) {
    analyzers_.erase(it);
    return true;
  }
  return false;
}

bool AnalyzerGroup::match(const string name)
{
  if (analyzers_.size() == 0) {
    return false;
  }

  bool match_name = false;
  if (matched_.count(name)) {
    vector<bool> & mtch_vec = matched_[name];
    for (unsigned int i = 0; i < mtch_vec.size(); ++i) {
      if (mtch_vec[i]) {
        return true;
      }
    }
    return false;
  }

  matched_[name].resize(analyzers_.size());
  for (unsigned int i = 0; i < analyzers_.size(); ++i) {
    bool mtch = analyzers_[i]->match(name);
    match_name = mtch || match_name;
    matched_[name].at(i) = mtch;
  }

  return match_name;
}

void AnalyzerGroup::resetMatches() {matched_.clear();}

bool AnalyzerGroup::analyze(const std::shared_ptr<StatusItem> item)
{
  bool analyzed = false;
  vector<bool> & mtch_vec = matched_[item->getName()];
  for (unsigned int i = 0; i < mtch_vec.size(); ++i) {
    if (mtch_vec[i]) {
      analyzed = analyzers_[i]->analyze(item) || analyzed;
    }
  }

  return analyzed;
}

vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>>
AnalyzerGroup::report()
{
  vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> output;

  std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus> header_status(
    new diagnostic_msgs::msg::DiagnosticStatus);
  header_status->name = path_;
  header_status->level = 0;
  header_status->message = "OK";

  if (analyzers_.size() == 0) {
    header_status->level = 2;
    header_status->message = "No analyzers";
    output.push_back(header_status);

    if (header_status->name == "" || header_status->name == "/") {
      header_status->name = "/AnalyzerGroup";
    }

    return output;
  }

  bool all_stale = true;

  for (unsigned int j = 0; j < analyzers_.size(); ++j) {
    string path = analyzers_[j]->getPath();
    string nice_name = analyzers_[j]->getName();

    vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> processed =
      analyzers_[j]->report();

    // Do not report anything in the header values for analyzers that don't
    // report
    if (processed.size() == 0) {
      continue;
    }

    // Look through processed data for header, append it to header_status
    // Ex: Look for /Robot/Power and append (Power, OK) to header
    for (unsigned int i = 0; i < processed.size(); ++i) {
      output.push_back(processed[i]);

      // Add to header status
      if (processed[i]->name == path) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = nice_name;
        kv.value = processed[i]->message;

        all_stale = all_stale && (processed[i]->level == 3);
        header_status->level = max(header_status->level, processed[i]->level);
        header_status->values.push_back(kv);
      }
    }
  }

  // Report stale as errors unless all stale
  if (header_status->level == 3 && !all_stale) {
    header_status->level = 2;
  }

  header_status->message = valToMsg(header_status->level);

  if (path_ != "" && path_ != "/") {  // No header if we don't have a base path
    output.push_back(header_status);
  }

  for (unsigned int i = 0; i < aux_items_.size(); ++i) {
    output.push_back(aux_items_[i]->toStatusMsg(path_, true));
  }

  return output;
}
//}
//}
