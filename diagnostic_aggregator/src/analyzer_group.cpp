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

#include "diagnostic_aggregator/analyzer_group.hpp"

using namespace std;
using namespace diagnostic_aggregator;

using rclcpp::get_logger;

PLUGINLIB_EXPORT_CLASS(diagnostic_aggregator::AnalyzerGroup,
  diagnostic_aggregator::Analyzer)

AnalyzerGroup::AnalyzerGroup()
: path_(""), nice_name_(""),
  analyzer_loader_("diagnostic_aggregator", "diagnostic_aggregator::Analyzer")
{}

bool AnalyzerGroup::init(const string base_path, const rclcpp::Node::SharedPtr n)
{
  RCLCPP_DEBUG(get_logger("AnalyzerGroup"), "init()");
  bool init_ok = true;
  
  std::string path = base_path;
  if (!base_path.empty()) {
    path += ".analyzers";
  }  
  
  std::map<std::string, rclcpp::Parameter> parameters;
  if (!n->get_parameters(path, parameters)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("AnalyzerGroup"),
      "Couldn't retrieve parameters for analyzer group '%s', namespace '%s'.",
      path.c_str(), n->get_namespace());
      return false;
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("AnalyzerGroup"),
    "Retrieved %d parameter(s) for analyzer group '%s.",
    parameters.size(), path.c_str());

  std::string ns, an_type, an_path;
  std::shared_ptr<Analyzer> analyzer;
  for (auto & param : parameters) {
    RCLCPP_DEBUG(rclcpp::get_logger("AnalyzerGroup"), "Analyzer parameter %s:%s:%s", param.first.c_str(), param.second.get_type_name().c_str(), param.second.value_to_string().c_str());
    
    std::string tmpname = param.first.substr(0, param.first.find("."));
    if (tmpname.compare(ns) != 0) {
      // New analyzer, first create the previous one
      if (!ns.empty() && !an_type.empty() && !an_path.empty()) {
       
        RCLCPP_WARN(rclcpp::get_logger("AnalyzerGroup"), "Initialize %s '%s' for %s", an_type.c_str(), ns.c_str(), an_path.c_str()); 
        bool have_class = false;
        
        try {
          if (!analyzer_loader_.isClassAvailable(an_type)) {
            RCLCPP_ERROR(get_logger("AnalyzerGroup"), "Unable to find Analyzer class %s. Check that Analyzer is fully declared.",
              an_type.c_str());
          }
          
          analyzer = analyzer_loader_.createSharedInstance(an_type);         
        } catch (pluginlib::LibraryLoadException & e) {
          RCLCPP_ERROR(get_logger("AnalyzerGroup"), "Failed to load analyzer %s, type %s. Caught exception: %s",
            ns.c_str(), an_type.c_str(), e.what());
          std::shared_ptr<StatusItem> item(new StatusItem(ns,
            "Pluginlib exception loading analyzer"));
          aux_items_.push_back(item);
          init_ok = false;
          continue;
        }
        
        if (!analyzer) {
          RCLCPP_ERROR(get_logger("AnalyzerGroup"), "Pluginlib returned a null analyzer for %s, namespace %s.",
            an_type.c_str(), n->get_namespace());
          std::shared_ptr<StatusItem> item(new StatusItem(ns,
            "Pluginlib return NULL Analyzer for " +
            an_type));
          aux_items_.push_back(item);
          init_ok = false;
          continue;
        }
        
        if (path.empty()) {
          an_path = ns;
        } else {
          an_path = path + "." + ns;
        }
        RCLCPP_DEBUG(get_logger("AnalyzerGroup"), " an_path: %s",
            an_path.c_str());
        if (!analyzer->init(an_path, n)) {
          RCLCPP_ERROR(get_logger("AnalyzerGroup"), "Unable to initialize analyzer NS: %s, type: %s",
            n->get_namespace(), an_type.c_str());
          std::shared_ptr<StatusItem> item(new StatusItem(ns, "Analyzer init failed"));
          aux_items_.push_back(item);
          init_ok = false;
          continue;
        }

        this->addAnalyzer(analyzer); 
      }
      
      ns = tmpname; an_type = ""; an_path = "";
      RCLCPP_DEBUG(rclcpp::get_logger("AnalyzerGroup"), "New analyzer: %s", ns.c_str());
    }    
    std::stringstream p_type;
    p_type << ns << ".type";
    if (param.first.compare(p_type.str()) == 0) {
      an_type = param.second.value_to_string();
      RCLCPP_DEBUG(rclcpp::get_logger("AnalyzerGroup"), "Analyzer type: %s", an_type.c_str());
    }
    std::stringstream p_path;
    p_path << ns << ".path";
    if (param.first.compare(p_path.str()) == 0) {
      an_path = param.second.value_to_string();
      RCLCPP_DEBUG(rclcpp::get_logger("AnalyzerGroup"), "Analyzer path: %s", an_path.c_str());
    }
  }

  if (analyzers_.size() == 0) {
    init_ok = false;
    RCLCPP_ERROR(get_logger("AnalyzerGroup"), "No analyzers initialized in AnalyzerGroup '%s'", n->get_namespace());
  }

  return init_ok;
}

AnalyzerGroup::~AnalyzerGroup()
{
  RCLCPP_DEBUG(get_logger("AnalyzerGroup"), "destructor");
  analyzers_.clear();
}

bool AnalyzerGroup::addAnalyzer(std::shared_ptr<Analyzer> & analyzer)
{
  RCLCPP_DEBUG(get_logger("AnalyzerGroup"), "addAnalyzer()");
  analyzers_.push_back(analyzer);
  return true;
}

bool AnalyzerGroup::removeAnalyzer(std::shared_ptr<Analyzer> & analyzer)
{
  RCLCPP_DEBUG(get_logger("AnalyzerGroup"), "removeAnalyzer()");
  vector<std::shared_ptr<Analyzer>>::iterator it = find(analyzers_.begin(),
      analyzers_.end(), analyzer);
  if (it != analyzers_.end()) {
    analyzers_.erase(it);
    return true;
  }
  return false;
}

bool AnalyzerGroup::match(const string name)
{
  RCLCPP_DEBUG(get_logger("AnalyzerGroup"), "match()");
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

void AnalyzerGroup::resetMatches()
{
  RCLCPP_DEBUG(get_logger("AnalyzerGroup"), "resetMatches()");
  matched_.clear();
}


bool AnalyzerGroup::analyze(const std::shared_ptr<StatusItem> item)
{
  RCLCPP_DEBUG(get_logger("AnalyzerGroup"), "analyze()");
  /* @todo(anordman):assertion ROS_ASSERT_MSG(get_logger(), matched_.count(
      item->getName()), "AnalyzerGroup was asked to analyze an item it hadn't matched.");*/

  bool analyzed = false;
  vector<bool> & mtch_vec = matched_[item->getName()];
  for (unsigned int i = 0; i < mtch_vec.size(); ++i) {
    if (mtch_vec[i]) {
      analyzed = analyzers_[i]->analyze(item) || analyzed;
    }
  }

  return analyzed;
}

vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> AnalyzerGroup::report()
{
  RCLCPP_DEBUG(get_logger("AnalyzerGroup"), "report()");
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

    // Do not report anything in the header values for analyzers that don't report
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

  if (path_ != "" && path_ != "/") { // No header if we don't have a base path
    output.push_back(header_status);
  }

  for (unsigned int i = 0; i < aux_items_.size(); ++i) {
    output.push_back(aux_items_[i]->toStatusMsg(path_, true));
  }

  return output;
}
