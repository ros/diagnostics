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

///\author Kevin Watts

#include "diagnostic_aggregator/component_analyzer.h"

using namespace diagnostic_aggregator;
using namespace std;

ComponentAnalyzer::ComponentAnalyzer() : timeout_(5.0) { }

bool ComponentAnalyzer::init(string first_prefix, const ros::NodeHandle &n)
{ 
  XmlRpc::XmlRpcValue params;
  if (!n.getParam("", params))
  {
    ROS_FATAL("ComponentAnalyzer was not given any parameters.");
    ROS_BREAK();
  }

  if (!n.getParam("prefix", nice_name_))
  {
    ROS_FATAL("ComponentAnalyzer was not given parameter \"prefix\".");
    ROS_BREAK();
  }
  full_prefix_ = first_prefix + "/" + nice_name_;

  n.param("timeout", timeout_, timeout_);

   // Create components from list
  XmlRpc::XmlRpcValue::iterator xml_it;
  for (xml_it = params.begin(); xml_it != params.end(); ++xml_it)
  {
    XmlRpc::XmlRpcValue param_name = xml_it->first;
    ROS_DEBUG("Got param \"name\": %s", param_name.toXml().c_str());
    string p_name = param_name;

    // special parameter names
    if (p_name == "type" || p_name == "prefix" || 
        p_name == "timeout")
      continue;

    XmlRpc::XmlRpcValue component = xml_it->second;
    if (!component.hasMember("name"))
    {
      ROS_FATAL("Component %s has no member \"name\", unable to initialize PR2Component", p_name.c_str());
      ROS_BREAK();
    }
    XmlRpc::XmlRpcValue component_name = component["name"];
    string name = component_name;

    if (!component.hasMember("fields"))
    {
      ROS_FATAL("Component %s has no member 'fields', unable to initialize PR2Component.", p_name.c_str());
      ROS_BREAK();
    }
    XmlRpc::XmlRpcValue fields = component["fields"];

    if (!component.hasMember("start_name"))
    {
      boost::shared_ptr<Component> pr2_comp(new Component(full_prefix_, name, timeout_, "", fields));
    components_.push_back(pr2_comp);
    }
    else
    {
      XmlRpc::XmlRpcValue start_str = component["start_name"];
      string start_name = start_str;
      
      boost::shared_ptr<Component> pr2_comp(new Component(full_prefix_, name, timeout_, start_name, fields));
      components_.push_back(pr2_comp);
    }
  }

  if (components_.size() == 0)
  {
    ROS_FATAL("ComponentAnalyzer was not given any sensors to analyze.");
    ROS_BREAK();
  }
 
  return true;
}

ComponentAnalyzer::~ComponentAnalyzer() 
{
  components_.clear();
}

vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > ComponentAnalyzer::analyze(map<string, boost::shared_ptr<StatusItem> > msgs)
{
  boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> header_status(new diagnostic_msgs::DiagnosticStatus());
  header_status->name = full_prefix_;
  header_status->level = 0;
  header_status->message = "OK";

  vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > processed;
  processed.push_back(header_status);

  bool all_stale = true;
  
  for (unsigned int i = 0; i < components_.size(); ++i)
  {
    boost::shared_ptr<Component> component = components_[i];
    
    vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > my_components = component->analyze(msgs);
    
    for (unsigned int j = 0; j < my_components.size(); ++j)
      processed.push_back(my_components[j]);

    diagnostic_msgs::KeyValue kv;
    kv.key = component->getName();
    kv.value = component->getHeaderMsg();
    header_status->values.push_back(kv);

    int8_t level = component->getHeaderLevel();
    all_stale = all_stale && level == 3;

    header_status->level = max(header_status->level, level);
  }

  if (all_stale)
    header_status->level = 3;
  else if (header_status->level == 3)
    header_status->level = 2; 
  
  header_status->message = valToMsg(header_status->level);

  return processed;
}
