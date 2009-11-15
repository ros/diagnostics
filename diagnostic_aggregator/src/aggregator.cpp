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

#include <diagnostic_aggregator/aggregator.h>

using namespace std;
using namespace diagnostic_aggregator;

Aggregator::Aggregator() :
  pub_rate_(1.0),
  analyzer_loader_("diagnostic_aggregator", "diagnostic_aggregator::Analyzer")
{
  ros::NodeHandle nh = ros::NodeHandle("~");
  nh.param(string("base_path"), base_path_, string(""));
  if (base_path_.size() > 0 && base_path_.find("/") != 0)
    base_path_ = "/" + base_path_;

  nh.param("pub_rate", pub_rate_, pub_rate_);
    
  XmlRpc::XmlRpcValue private_params;
  nh.getParam("analyzers", private_params);
  ROS_DEBUG("Private params: %s.", private_params.toXml().c_str());

  XmlRpc::XmlRpcValue::iterator xml_it;
  for (xml_it = private_params.begin(); xml_it != private_params.end(); ++xml_it)
  {
    XmlRpc::XmlRpcValue analyzer_name = xml_it->first;
    ROS_DEBUG("Got analyzer name: %s", analyzer_name.toXml().c_str());
    XmlRpc::XmlRpcValue analyzer_value = xml_it->second;

    string ns = analyzer_name;
    string full_ns = "analyzers/" + ns;
    
    if (!analyzer_value.hasMember("type"))
    {
      ROS_ERROR("Namespace %s has no member 'type', unable to initialize analyzer for this namespace.", full_ns.c_str());
      boost::shared_ptr<StatusItem> item(new StatusItem(ns, "No Analyzer type given"));
      aux_items_.push_back(item);
      continue;
    }

    XmlRpc::XmlRpcValue analyzer_type = analyzer_value["type"];
    string an_type = analyzer_type;
    
    Analyzer* analyzer = NULL;
    try
    {
      analyzer = analyzer_loader_.createClassInstance(an_type);
    }
    catch (pluginlib::LibraryLoadException& e)
    {
      ROS_ERROR("Failed to load analyzer %s, type %s. Caught exception. %s", ns.c_str(), an_type.c_str(), e.what());
      boost::shared_ptr<StatusItem> item(new StatusItem(ns, "Pluginlib exception loading analyzer"));
      aux_items_.push_back(item);
      continue;
    }

    if (analyzer == NULL)
    {
      ROS_ERROR("Pluginlib returned a null analyzer for %s, namespace %s.", an_type.c_str(), full_ns.c_str());
      boost::shared_ptr<StatusItem> item(new StatusItem(ns, "Pluginlib return NULL Analyzer for " + an_type));
      aux_items_.push_back(item);
      continue;
    }
    
    if (!analyzer->init(base_path_, ros::NodeHandle(nh, full_ns)))
    {
      ROS_ERROR("Unable to initialize analyzer NS: %s, type: %s", full_ns.c_str(), an_type.c_str());
      boost::shared_ptr<StatusItem> item(new StatusItem(ns, "Analyzer init failed"));
      aux_items_.push_back(item);
      continue;
    }
    
    analyzers_.push_back(analyzer);
  }

  // Last analyzer handles remaining data
  other_analyzer_ = new OtherAnalyzer();
  other_analyzer_->init(base_path_);

  diag_sub_ = n_.subscribe("/diagnostics", 1000, &Aggregator::diagCallback, this);
  agg_pub_ = n_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics_agg", 1);
}

void Aggregator::diagCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& diag_msg)
{
  bool analyzed = false;
  for (unsigned int j = 0; j < diag_msg->status.size(); ++j)
  {
    boost::shared_ptr<StatusItem> item(new StatusItem(&diag_msg->status[j]));
    analyzed = false;
    for (unsigned int i = 0; i < analyzers_.size(); ++i)
    {
      if (analyzers_[i]->match(item->getName()))
    	  analyzed = analyzed || analyzers_[i]->analyze(item);
    }
    if (!analyzed)
      other_analyzer_->analyze(item);
  }
}

Aggregator::~Aggregator() 
{
  for (unsigned int i = 0; i < analyzers_.size(); ++i)
    delete analyzers_[i];

  analyzers_.clear();
}

void Aggregator::publishData()
{
  diagnostic_msgs::DiagnosticArray array;

  diagnostic_msgs::DiagnosticStatus header_status;
  header_status.name = base_path_;
  header_status.level = 0;
  header_status.message = "OK";

  // Call analyzers on data
  for (unsigned int j = 0; j < analyzers_.size(); ++j)
  {
    string path = analyzers_[j]->getPath();
    string nice_name = analyzers_[j]->getName();

    vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > processed = analyzers_[j]->report();

    // Look through processed data for header, append it to header_status
    // Ex: Look for /Robot/Power and append (Power, OK) to header
    for (unsigned int i = 0; i < processed.size(); ++i)
    {
      array.status.push_back(*processed[i]);

      // Add to header status
      if (processed[i]->name == path)
      {
        diagnostic_msgs::KeyValue kv;
        kv.key = nice_name;
        kv.value = processed[i]->message;

        header_status.level = max(header_status.level, processed[i]->level);
        header_status.values.push_back(kv);
      }
    }
  }

  vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > processed = other_analyzer_->report();
  for (unsigned int i = 0; i < processed.size(); ++i)
	  array.status.push_back(*processed[i]);

  header_status.message = valToMsg(header_status.level);

  if (base_path_ != "") // No header if we don't have a base path
    array.status.push_back(header_status);

  for (unsigned int i = 0; i < aux_items_.size(); ++i)
  {
    boost::shared_ptr<diagnostic_msgs::DiagnosticStatus>  stat = aux_items_[i]->toStatusMsg(base_path_, true);
    array.status.push_back(*stat);
  }

  array.header.stamp = ros::Time::now();

  agg_pub_.publish(array);
}
  

  

    



