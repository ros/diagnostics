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

GenericAnalyzer::GenericAnalyzer() { }

bool GenericAnalyzer::init(const string base_path, const ros::NodeHandle &n)
{ 
	string nice_name;
  if (!n.getParam("path", nice_name))
  {
    ROS_ERROR("GenericAnalyzer was not given parameter \"path\".");
    return false;
  }

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
	      addItem(expected_str, item);
	  }
	  else if (type == XmlRpc::XmlRpcValue::TypeArray)
	  {
		  for (int i = 0; i < expected.size(); ++i)
		  {
			  string expected_str = expected[i];
			  expected_.push_back(expected_str);

		      boost::shared_ptr<StatusItem> item(new StatusItem(expected_str));
		      addItem(expected_str, item);
		  }
	  }
	  else
		  ROS_WARN("Parameter \"expected\" was not a list or string for Analyzer %s", nice_name_.c_str());
  }

  vector<string> regex_strs;
  XmlRpc::XmlRpcValue regexes;
  if (n.getParam("regex", regexes))
  {
	  XmlRpc::XmlRpcValue::Type type = regexes.getType();
	  if (type == XmlRpc::XmlRpcValue::TypeString)
	  {
		  string regex_str = regexes;
		  regex_strs.push_back(regex_str);
	  }
	  else if (type == XmlRpc::XmlRpcValue::TypeArray)
	  {
		  for (int i = 0; i < regexes.size(); ++i)
		  {
			  string regex_str = regexes[i];
			  regex_strs.push_back(regex_str);
		  }
	  }
	  else
		  ROS_WARN("Parameter \"regex\" was not a list or string for Analyzer %s", nice_name_.c_str());
  }

  for (unsigned int i = 0; i < regex_strs.size(); ++i)
  {
	  try
      {
		  boost::regex re(regex_strs[i]);
		  regex_.push_back(re);
      }
      catch (boost::regex_error& e)
      {
    	  ROS_WARN("Attempted to make regex from %s. Caught exception, ignoring value. %s", regex_strs[i].c_str(), e.what());
      }
  }

  double timeout;
  int num_items_expected;
  n.param("timeout", timeout, 5.0);   // Timeout for stale
  n.param("num_items", num_items_expected, -1); // Number of items must match this

  return GenericAnalyzerBase::init(base_path + "/" + nice_name, nice_name, timeout, num_items_expected);
}

GenericAnalyzer::~GenericAnalyzer() { }


bool GenericAnalyzer::match(const string name) const
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
