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

#include "diagnostic_aggregator/generic_analyzer.h"

using namespace diagnostic_aggregator;
using namespace std;

PLUGINLIB_REGISTER_CLASS(GenericAnalyzer, diagnostic_aggregator::GenericAnalyzer, 
                         diagnostic_aggregator::Analyzer)

GenericAnalyzer::GenericAnalyzer() { }

bool GenericAnalyzer::init(const string base_path, const ros::NodeHandle &n)
{ 
  string nice_name;
  if (!n.getParam("path", nice_name))
  {
    ROS_ERROR("GenericAnalyzer was not given parameter \"path\". Namepspace: %s",
              n.getNamespace().c_str());
    return false;
  }

  XmlRpc::XmlRpcValue findRemove;
  if (n.getParam("find_and_remove_prefix", findRemove))
  {
    vector<string> output;
    getParamVals(findRemove, output);
    chaff_ = output;
    startswith_ = output;
  }
  
  XmlRpc::XmlRpcValue removes;
  if (n.getParam("remove_prefix", removes))
    getParamVals(removes, chaff_);
    
  XmlRpc::XmlRpcValue startswith;
  if (n.getParam("startswith", startswith))
    getParamVals(startswith, startswith_);

  XmlRpc::XmlRpcValue name_val;
  if (n.getParam("name", name_val))
    getParamVals(name_val, name_);

  XmlRpc::XmlRpcValue contains;
  if (n.getParam("contains", contains))
    getParamVals(contains, contains_);

  XmlRpc::XmlRpcValue expected;
  if (n.getParam("expected", expected))
  {
    getParamVals(expected, expected_);
    for (unsigned int i = 0; i < expected_.size(); ++i)
    {
      boost::shared_ptr<StatusItem> item(new StatusItem(expected_[i]));
      addItem(expected_[i], item);
    }
 }
 
  XmlRpc::XmlRpcValue regexes;
  if (n.getParam("regex", regexes))
  {
    vector<string> regex_strs;
    getParamVals(regexes, regex_strs);
  
    for (unsigned int i = 0; i < regex_strs.size(); ++i)
    {
      try
      {
        boost::regex re(regex_strs[i]);
        regex_.push_back(re);
      }
      catch (boost::regex_error& e)
      {
        ROS_WARN("Attempted to make regex from %s. Caught exception, ignoring value. Exception: %s", 
                 regex_strs[i].c_str(), e.what());
      }
    }
  }

  if (startswith_.size() == 0 && name_.size() == 0 && 
      contains_.size() == 0 && expected_.size() == 0 && regex_.size() == 0)
  {
    ROS_ERROR("GenericAnalyzer was not initialized with any way of checking diagnostics. Name: %s, namespace: %s", nice_name.c_str(), n.getNamespace().c_str());
    return false;
  }
  
  double timeout;
  int num_items_expected;
  bool discard_stale;
  n.param("timeout", timeout, 5.0);   // Timeout for stale
  n.param("num_items", num_items_expected, -1); // Number of items must match this
  n.param("discard_stale", discard_stale, false);

  return GenericAnalyzerBase::init(base_path + "/" + nice_name, nice_name, 
                                   timeout, num_items_expected, discard_stale);
}

GenericAnalyzer::~GenericAnalyzer() { }


bool GenericAnalyzer::match(const string name)
{
  boost::cmatch what;
  for (unsigned int i = 0; i < regex_.size(); ++i)
  {
    if (boost::regex_match(name.c_str(), what, regex_[i]))
      return true;
  }
  
  for (unsigned int i = 0; i < expected_.size(); ++i)
  {
    if (name == expected_[i])
      return true;
  }
  
  for (unsigned int i = 0; i < name_.size(); ++i)
  {
    if (name == name_[i])
      return true;
  }
  
  for (unsigned int i = 0; i < startswith_.size(); ++i)
  {
    if (name.find(startswith_[i]) == 0)
      return true;
  }
  
  for (unsigned int i = 0; i < contains_.size(); ++i)
  {
    if (name.find(contains_[i]) != string::npos)
      return true;
  }
  
  return false;
}

vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > GenericAnalyzer::report()
{
  vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > processed = GenericAnalyzerBase::report();
  
  for (unsigned int j = 0; j < processed.size(); ++j)
  {
    for (unsigned int i = 0; i < chaff_.size(); ++i)
      processed[j]->name = removeLeadingNameChaff(processed[j]->name, chaff_[i]);
  }
  
  return processed;
}
