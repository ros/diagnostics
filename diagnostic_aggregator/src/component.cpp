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

#include "diagnostic_aggregator/component.h"

using namespace std;
using namespace diagnostic_aggregator;


Component::Component(string prefix, string name, double timeout)
{
  nice_name_ = name;
  prefix_ = prefix + "/" + name;
  timeout_ = timeout;
}

Component::Component(string prefix, string name, double timeout, string start_name, XmlRpc::XmlRpcValue params) 
{
  nice_name_ = name;
  prefix_ = prefix + "/" + name;
  timeout_ = timeout;

  start_name_ = start_name;
  
  for (int i = 0; i < params.size(); ++i)
  {
    std::string field_name = params[i];
    boost::shared_ptr<StatusItem> item(new StatusItem(field_name));
    items_[field_name] = item;
  }
}

void Component::updateItems(std::map<string, boost::shared_ptr<StatusItem> > msgs)
{
  if (items_.size() == 0)
    ROS_WARN("No items to analyze for analyzer %s, component %s. It may have been initialized badly.", prefix_.c_str(), nice_name_.c_str());

  map<string, boost::shared_ptr<StatusItem> >::iterator item_it;

  map<string, boost::shared_ptr<StatusItem> >::iterator msgs_it;
  for (msgs_it = msgs.begin(); msgs_it != msgs.end(); ++msgs_it)
  {
    boost::shared_ptr<StatusItem> item = msgs_it->second;
    string name = msgs_it->first;

    item_it = items_.find(name);
    if (item_it != items_.end())
      items_[name] = item;
  }
}

vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > Component::analyze(std::map<string, boost::shared_ptr<StatusItem> > msgs)
{
  updateItems(msgs);

  bool all_stale = true;

  boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> header_status(new diagnostic_msgs::DiagnosticStatus());
  header_status->name = prefix_;
  header_status->level = 0;
  header_status->message = "OK";

  vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > processed;
  processed.push_back(header_status);

  map<string, boost::shared_ptr<StatusItem> >::iterator it;
  for (it = items_.begin(); it != items_.end(); it++)
  {
    string name = it->first;
    boost::shared_ptr<StatusItem> item = it->second;

    diagnostic_msgs::KeyValue kv;
    kv.key = name;
    kv.value = item->getMessage();

    header_status->values.push_back(kv);

    bool stale = item->getUpdateInterval().toSec() > timeout_; 

    boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> stat = item->toStatusMsg(prefix_, stale);

    processed.push_back(stat);

    header_status->level = max(header_status->level, stat->level);

    all_stale = all_stale && ((stat->level == 3) || stale);

    if (stale)
      header_status->level = 2;

    stat->name = removeLeadingNameChaff(stat->name, start_name_);
  }

  if (all_stale)
    header_status->level = 3;
  else if (header_status->level == 3)
    header_status->level = 2; 

  if (header_status->level == 1)
    header_status->message = "Warning";
  if (header_status->level == 2)
    header_status->message = "Error";
  if (header_status->level == 3)
    header_status->message = "All Stale";

  header_level_ = header_status->level;
  header_msg_ = header_status->message;

  return processed;
}
