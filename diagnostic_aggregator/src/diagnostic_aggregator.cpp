/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

// Author: Kevin Watts

#include <diagnostic_aggregator/diagnostic_aggregator.h>

using namespace std;
using namespace diagnostic_aggregator;


DiagnosticAggregator::DiagnosticAggregator(std::string prefix) : 
  analyzer_loader_("diagnostic_aggregator", "diagnostic_analyzer::DiagnosticAnalyzer")
{
  prefix_ = prefix;
  
  if (prefix_.find("/") != 0)
    prefix_ = "/" + prefix_;
}

DiagnosticAggregator::~DiagnosticAggregator() 
{
  clearMessages();

  for (unsigned int i = 0; i < analyzers_.size(); ++i)
    delete analyzers_[i];

  analyzers_.clear();
}

void DiagnosticAggregator::init() 
{
  diag_sub_ = n_.subscribe("/diagnostics", 1000, &DiagnosticAggregator::diagCallback, this);
  agg_pub_ = n_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics_agg", 1);

  // Initialize all analyzers.
  // Give node handle in "prefix" namespace
  XmlRpc::XmlRpcValue private_params;
  n_.getParam("~", private_params);
  ROS_DEBUG("Private params: %s.", private_params.toXml().c_str());

  XmlRpc::XmlRpcValue::iterator xml_it;

  for (xml_it = private_params.begin(); xml_it != private_params.end(); ++xml_it)
  {
    XmlRpc::XmlRpcValue analyzer_name = xml_it->first;
    ROS_DEBUG("Got analyzer name: %s", analyzer_name.toXml().c_str());
    XmlRpc::XmlRpcValue analyzer_value = xml_it->second;

    string ns = analyzer_name;
    
    if (!analyzer_value.hasMember("type"))
    {
      ROS_FATAL("Namespace %s has no member 'type', unable to initialize analyzer", ns.c_str());
      ROS_BREAK();
    }

    XmlRpc::XmlRpcValue analyzer_type = analyzer_value["type"];
    string an_type = analyzer_type;
    
    diagnostic_analyzer::DiagnosticAnalyzer* analyzer = analyzer_loader_.createClassInstance(an_type);
    if (analyzer == NULL)
    {
      ROS_FATAL("Pluginlib returned a null analyzer for %s, namespace %s.", an_type.c_str(), ns.c_str());
      ROS_BREAK();
    }
    
    if (!analyzer->init(prefix_, ros::NodeHandle(n_, ns)))
    {
      ROS_FATAL("Unable to initialize analyzer NS: %s, type: %s", ns.c_str(), an_type.c_str());
      ROS_BREAK();
    }
    
    analyzers_.push_back(analyzer);
  }

  // Last analyzer handles remaining data
  diagnostic_analyzer::GenericAnalyzer *remainder = new diagnostic_analyzer::GenericAnalyzer();
  remainder->initOther(prefix_);
  
  analyzers_.push_back(remainder);
}

void DiagnosticAggregator::diagCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& diag_msg)
{
  map<string, diagnostic_item::DiagnosticItem*>::iterator it;
  for (unsigned int i = 0; i < diag_msg->status.size(); ++i)
  {
    it = msgs_.find(diag_msg->status[i].name);
    if (it == msgs_.end())
      msgs_[diag_msg->status[i].name] = new diagnostic_item::DiagnosticItem(&diag_msg->status[i]);
    else
      msgs_[diag_msg->status[i].name]->update(&diag_msg->status[i]);
  }
}


void DiagnosticAggregator::clearMessages()
{
  map<string, diagnostic_item::DiagnosticItem*>::iterator it;

  for (it = msgs_.begin(); it != msgs_.end(); ++it)
  {
    diagnostic_item::DiagnosticItem *ptr = (*it).second;
    delete ptr;
  }
  msgs_.clear();
}


void DiagnosticAggregator::publishData()
{
  diagnostic_msgs::DiagnosticArray array;

  diagnostic_msgs::DiagnosticStatus header_status;
  header_status.name = prefix_;
  header_status.level = 0;
  header_status.message = "OK";

  // Call analyzers on data
  for (unsigned int j = 0; j < analyzers_.size(); ++j)
  {
    string prefix = analyzers_[j]->getPrefix();
    string nice_name = analyzers_[j]->getName();

    vector<diagnostic_msgs::DiagnosticStatus*> processed = analyzers_[j]->analyze(msgs_);

    // Look through processed data for header, append it to header_status
    // Ex: Look for /Robot/Power and append (Power, OK) to header
    for (unsigned int i = 0; i < processed.size(); ++i)
    {
      array.status.push_back(*processed[i]);

      // Add to header status
      if (processed[i]->name == prefix)
      {
        diagnostic_msgs::KeyValue kv;
        kv.key = nice_name;
        kv.value = processed[i]->message;

        header_status.level = max(header_status.level, processed[i]->level);
        header_status.values.push_back(kv);
      }

      delete processed[i];
    }
    processed.clear();
  }

  if (header_status.level == 1)
    header_status.message = "Warning";
  if (header_status.level == 2)
    header_status.message = "Error";
  if (header_status.level == 3)
    header_status.message = "Error";


  array.status.push_back(header_status);

  agg_pub_.publish(array);

  clearMessages();
}
  

  

    



