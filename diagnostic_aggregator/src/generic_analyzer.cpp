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

PLUGINLIB_REGISTER_CLASS(GenericAnalyzer, diagnostic_aggregator::GenericAnalyzer, diagnostic_aggregator::Analyzer)

GenericAnalyzer::GenericAnalyzer() : timeout_(5.0), num_items_expected_(-1) { }

bool GenericAnalyzer::init(const string base_path, const ros::NodeHandle &n)
{ 
  if (!n.getParam("path", nice_name_))
  {
    ROS_ERROR("GenericAnalyzer was not given parameter \"path\".");
    return false;
  }
  base_path_ = base_path + "/" + nice_name_;

  XmlRpc::XmlRpcValue findRemove;
  if (n.getParam("find_and_remove_prefix", findRemove))
  {
	  XmlRpc::XmlRpcValue::Type type = findRemove.getType();
	  if (type == XmlRpc::XmlRpcValue::TypeString)
	  {
		  string find = findRemove;
		  startswith_.push_back(find);
		  chaff_.push_back(find);
	  }
	  else if (type == XmlRpc::XmlRpcValue::TypeArray)
	  {
		  for (int i = 0; i < findRemove.size(); ++i)
		  {
			  string find = findRemove[i];
			  startswith_.push_back(find);
			  chaff_.push_back(find);
		  }
	  }
	  else
		  ROS_WARN("Parameter \"find_and_remove_prefix\" was not a list or string for Analyzer %s", nice_name_.c_str());
  }

  XmlRpc::XmlRpcValue removes;
  if (n.getParam("remove_prefix", removes))
  {
	  XmlRpc::XmlRpcValue::Type type = removes.getType();
	  if (type == XmlRpc::XmlRpcValue::TypeString)
	  {
		  string find = removes;
		  chaff_.push_back(find);
	  }
	  else if (type == XmlRpc::XmlRpcValue::TypeArray)
	  {
		  for (int i = 0; i < removes.size(); ++i)
		  {
			  string find = removes[i];
			  chaff_.push_back(find);
		  }
	  }
	  else
		  ROS_WARN("Parameter \"remove_prefix\" was not a list or string for Analyzer %s", nice_name_.c_str());
  }


  XmlRpc::XmlRpcValue startswith;
  if (n.getParam("startswith", startswith))
  {
	  XmlRpc::XmlRpcValue::Type type = startswith.getType();
	  if (type == XmlRpc::XmlRpcValue::TypeString)
	  {
		  string sw = startswith;
		  startswith_.push_back(sw);
	  }
	  else if (type == XmlRpc::XmlRpcValue::TypeArray)
	  {
		  for (int i = 0; i < startswith.size(); ++i)
		  {
			  string sw = startswith[i];
			  startswith_.push_back(sw);
		  }
	  }
	  else
		  ROS_WARN("Parameter \"starts_with\" was not a list or string for Analyzer %s", nice_name_.c_str());
  }

  XmlRpc::XmlRpcValue name_val;
  if (n.getParam("name", name_val))
  {
	  XmlRpc::XmlRpcValue::Type type = name_val.getType();
	  if (type == XmlRpc::XmlRpcValue::TypeString)
	  {
		  string name = name_val;
		  name_.push_back(name);
	  }
	  else if (type == XmlRpc::XmlRpcValue::TypeArray)
	  {
		  for (int i = 0; i < name_val.size(); ++i)
		  {
			  string name = name_val[i];
			  name_.push_back(name);
		  }
	  }
	  else
		  ROS_WARN("Parameter \"name\" was not a list or string for Analyzer %s", nice_name_.c_str());
  }

  XmlRpc::XmlRpcValue contains;
  if (n.getParam("contains", contains))
  {
	  XmlRpc::XmlRpcValue::Type type = contains.getType();
	  if (type == XmlRpc::XmlRpcValue::TypeString)
	  {
		  string contain_str = contains;
		  contains_.push_back(contain_str);
	  }
	  else if (type == XmlRpc::XmlRpcValue::TypeArray)
	  {
		  for (int i = 0; i < contains.size(); ++i)
		  {
			  string contain_str = contains[i];
			  contains_.push_back(contain_str);
		  }
	  }
	  else
		  ROS_WARN("Parameter \"contains\" was not a list or string for Analyzer %s", nice_name_.c_str());
  }

  XmlRpc::XmlRpcValue expected;
  if (n.getParam("expected", expected))
  {
	  XmlRpc::XmlRpcValue::Type type = expected.getType();
	  if (type == XmlRpc::XmlRpcValue::TypeString)
	  {
		  string expected_str = expected;
		  expected_.push_back(expected_str);

	      boost::shared_ptr<StatusItem> item(new StatusItem(expected_str));
	      items_[expected_str] = item;
	  }
	  else if (type == XmlRpc::XmlRpcValue::TypeArray)
	  {
		  for (int i = 0; i < expected.size(); ++i)
		  {
			  string expected_str = expected[i];
			  expected_.push_back(expected_str);

		      boost::shared_ptr<StatusItem> item(new StatusItem(expected_str));
		      items_[expected_str] = item;
		  }
	  }
	  else
		  ROS_WARN("Parameter \"expected\" was not a list or string for Analyzer %s", nice_name_.c_str());
  }

  /*
  XmlRpc::XmlRpcValue regexes;
  if (n.getParam("regex", regexes))
  {
    for (int i = 0; i < regexes.size(); ++i)
    {
      string regex_str = regexes[i];
      try
      {
		  boost::regex re(regex_str);
		  regex_.push_back(re);
      }
      catch (boost::regex_error& e)
      {
    	  ROS_WARN("Attempted to make regex from %s. Caught exception, ignoring value. %s", regex_str.c_str(), e.what());
      }
    }
  }
*/

  n.param("timeout", timeout_, 5.0);   // Timeout for stale
  n.param("num_items", num_items_expected_, -1); // Number of items must match this

  return true;
}

GenericAnalyzer::~GenericAnalyzer()
{
  items_.clear();
}

bool GenericAnalyzer::match(const string name) const
{
	/*
	boost::cmatch what;
	for (unsigned int i = 0; i < regex_.size(); ++i)
	{
		if (boost::regex_match(name.c_str(), what, regex_[i]))
			return true;
	}
	*/

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

bool GenericAnalyzer::analyze(const boost::shared_ptr<StatusItem> item)
{
	items_[item->getName()] = item;
	return true;
}

vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > GenericAnalyzer::report()
{
  boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> header_status(new diagnostic_msgs::DiagnosticStatus());
  header_status->name = base_path_;
  header_status->level = 0;
  header_status->message = "OK";

  vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > processed;
  processed.push_back(header_status);

  bool all_stale = true;
  
  map<string, boost::shared_ptr<StatusItem> >::iterator it;
  for (it = items_.begin(); it != items_.end(); it++)
  {
    string name = it->first;
    boost::shared_ptr<StatusItem> item = it->second;

    int8_t level = item->getLevel();

    header_status->level = max(header_status->level, level);

    diagnostic_msgs::KeyValue kv;
    kv.key = name;
    kv.value = item->getMessage();
    
    header_status->values.push_back(kv);

    bool stale = (ros::Time::now() - item->getLastUpdateTime()).toSec() > timeout_;

    all_stale = all_stale && ((level == 3) || stale);

    boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> stat = item->toStatusMsg(base_path_, stale);

    for (unsigned int i = 0; i < chaff_.size(); ++i)
    	stat->name = removeLeadingNameChaff(stat->name, chaff_[i]);
    processed.push_back(stat);

    if (stale)
      header_status->level = 2;
  }
  
  // Header is not stale unless all subs are
  if (all_stale)
    header_status->level = 3;
  else if (header_status->level == 3)
    header_status->level = 2;

  header_status->message = valToMsg(header_status->level);

  if (num_items_expected_ > 0 and int(items_.size()) != num_items_expected_)
  {
	  int8_t lvl = 1;
	  header_status->level = max(lvl, header_status->level);
	  stringstream expec, item;
	  expec << num_items_expected_;
	  item << items_.size();
	  header_status->message = "Expected " + expec.str() + ", found " + item.str();
  }
  
  return processed;
}
