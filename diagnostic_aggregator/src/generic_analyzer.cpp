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

// Author: Kevin Watts

#include "diagnostic_aggregator/generic_analyzer.h"

using namespace diagnostic_analyzer;
using namespace std;

GenericAnalyzer::GenericAnalyzer() : other_(false) { }

bool GenericAnalyzer::initOther(string first_prefix)
{
  other_ = true;
  nice_name_ = "Other";
  full_prefix_ = first_prefix + "/" + nice_name_;

  ROS_DEBUG("Created remainder analyzer");
  return true;
}

bool GenericAnalyzer::init(string first_prefix, const ros::NodeHandle &n)
{ 
  if (!n.getParam("~prefix", nice_name_))
  {
    ROS_FATAL("GenericAnalyzer was not given parameter \"prefix\".");
    ROS_BREAK();
  }
  full_prefix_ = first_prefix + "/" + nice_name_;

  XmlRpc::XmlRpcValue startswith;
  if (n.getParam("~startswith", startswith))
  {
    for (int i = 0; i < startswith.size(); ++i)
    {
      string starts = startswith[i];
      startswith_.push_back(starts);
    }
  }

  XmlRpc::XmlRpcValue name_val;
  if (n.getParam("~name", name_val))
  {
    for (int i = 0; i < name_val.size(); ++i)
    {
      string name = name_val[i];
      name_.push_back(name);
    }
  }

  XmlRpc::XmlRpcValue contains;
  if (n.getParam("~contains", contains))
  {
    for (int i = 0; i < contains.size(); ++i)
    {
      string contain_str = contains[i];
      contains_.push_back(contain_str);
    }
  }

  XmlRpc::XmlRpcValue expected;
  if (n.getParam("~expected", expected))
  {
    for (int i = 0; i < expected.size(); ++i)
    {
      string expected_str = expected[i];
      expected_.push_back(expected_str);
      
      // Make sure we're looking for this item
      diagnostic_msgs::DiagnosticStatus *status = new diagnostic_msgs::DiagnosticStatus();
      status->name = expected_str;
      status->level = 2;
      status->message = "Missing";
      
      diagnostic_item::DiagnosticItem *item = new diagnostic_item::DiagnosticItem(status);
      items_[expected_str] = item;

      delete status;
    }
  }

  return true;
}

GenericAnalyzer::~GenericAnalyzer() 
{
  // Clear all items
  map<string, diagnostic_item::DiagnosticItem*>::iterator it;
  for (it = items_.begin(); it != items_.end(); ++it)
  {
    delete it->second;
    it->second = NULL;
  }
  items_.clear();
}

vector<diagnostic_msgs::DiagnosticStatus*> GenericAnalyzer::analyze(map<string, diagnostic_item::DiagnosticItem*> msgs)
{
  diagnostic_msgs::DiagnosticStatus *header_status = new diagnostic_msgs::DiagnosticStatus();
  header_status->name = full_prefix_;
  header_status->level = 0;
  header_status->message = "OK";

  // Output array, gets deleted by aggregator
  vector<diagnostic_msgs::DiagnosticStatus*> processed;
  processed.push_back(header_status);

  vector<diagnostic_msgs::DiagnosticStatus*> to_analyze;
  if (!other_)
    to_analyze = toAnalyze(msgs);
  else
    to_analyze = toAnalyzeOther(msgs);

  // Deletes items to analyze
  updateItems(to_analyze);

  bool all_stale = true;
  
  map<string, diagnostic_item::DiagnosticItem*>::iterator it;
  for (it = items_.begin(); it != items_.end(); it++)
  {
    string name = it->first;
    diagnostic_item::DiagnosticItem *item = it->second;

    int8_t level = item->getLevel();

    ///\todo NEED TO CHECK UPDATE TIME

    all_stale = all_stale && (level == 3);

    header_status->level = max(header_status->level, level);

    diagnostic_msgs::KeyValue kv;
    kv.key = name;
    kv.value = item->getMessage();
    
    header_status->values.push_back(kv);
    processed.push_back(item->toStatusMsg(full_prefix_, false));
  }
  if (header_status->level == 3 && !all_stale)
    header_status->level = 2;

  if (header_status->level == 1)
    header_status->message = "Warning";
  if (header_status->level == 2)
    header_status->message = "Error";
  if (header_status->level == 3)
    header_status->message = "All Stale";

  return processed;
}

void GenericAnalyzer::updateItems(vector<diagnostic_msgs::DiagnosticStatus*> to_analyze)
{
  map<string, diagnostic_item::DiagnosticItem*>::iterator it;

  for (unsigned int i = 0; i < to_analyze.size(); ++i)
  {
    it = items_.find(to_analyze[i]->name);
    if (it == items_.end())
      items_[to_analyze[i]->name] = new diagnostic_item::DiagnosticItem(to_analyze[i]);
    else
      items_[to_analyze[i]->name]->update(to_analyze[i]);
    
    delete to_analyze[i];
    to_analyze[i] = NULL;
  }
  to_analyze.clear();
}

// Returns vector of msgs that haven't been analyzed
vector<diagnostic_msgs::DiagnosticStatus*> GenericAnalyzer::toAnalyzeOther(map<string, diagnostic_item::DiagnosticItem*> msgs)
{
  vector<diagnostic_msgs::DiagnosticStatus*> to_analyze;
  
  map<string, diagnostic_item::DiagnosticItem*>::iterator it;

  for (it = msgs.begin(); it != msgs.end(); ++it)
  {
    if (!it->second->hasChecked())
      to_analyze.push_back(it->second->toStatusMsg());
  }

  return to_analyze;
}

// Returns vector of msgs to analyze
///\todo optimize with dictionaries or something
vector<diagnostic_msgs::DiagnosticStatus*> GenericAnalyzer::toAnalyze(map<string, diagnostic_item::DiagnosticItem*> msgs)
{
  vector<diagnostic_msgs::DiagnosticStatus*> to_analyze;
  
  map<string, diagnostic_item::DiagnosticItem*>::iterator it;

  for (it = msgs.begin(); it != msgs.end(); ++it)
  {
    // Look for all startswith, etc
    diagnostic_item::DiagnosticItem *item = it->second;
    
    string name = item->getName();

    bool analyzed = false;

    // Check expected, name, startswith, contains
    // If we're going to analyze it, don't check remainder
    for (unsigned int i = 0; i < expected_.size(); ++i)
    {
      if (name == expected_[i])
      {
        to_analyze.push_back(item->toStatusMsg());
        analyzed = true;
        break;
      }
    }
    if (analyzed)
      continue;

    for (unsigned int i = 0; i < name_.size(); ++i)
    {
      if (name == name_[i])
      {
        to_analyze.push_back(item->toStatusMsg());
        analyzed = true;
        break;
      }
    }
    if (analyzed)
      continue;

    for (unsigned int i = 0; i < startswith_.size(); ++i)
    {
      if (name.find(startswith_[i]) == 0)
      {
        to_analyze.push_back(item->toStatusMsg());
        analyzed = true;
        break;
      }
    }
    if (analyzed)
      continue;
 
    for (unsigned int i = 0; i < contains_.size(); ++i)
    {
      if (name.find(contains_[i]) != string::npos)
      {
        to_analyze.push_back(item->toStatusMsg());
        analyzed = true;
        break;
      }
    }
  }

  return to_analyze;
}


