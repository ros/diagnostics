/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

/*!
 * \author Kevin Watts 
 */

#include "test_diagnostic_aggregator/match_no_analyze_analyzer.h"

using namespace diagnostic_aggregator;
using namespace test_diagnostic_aggregator;
using namespace std;

PLUGINLIB_EXPORT_CLASS(test_diagnostic_aggregator::MatchNoAnalyzeAnalyzer,
                       diagnostic_aggregator::Analyzer)

MatchNoAnalyzeAnalyzer::MatchNoAnalyzeAnalyzer() :
  path_(""),
  nice_name_(""), 
  my_item_name_(""),
  has_initialized_(false)
{ }

MatchNoAnalyzeAnalyzer::~MatchNoAnalyzeAnalyzer() { }


bool MatchNoAnalyzeAnalyzer::init(const string base_name, const ros::NodeHandle &n)
{ 
  if (!n.getParam("path", nice_name_))
  {
     /* @todo(anordman):logging RCLCPP_ERROR(get_logger(), "No power board name was specified in MatchNoAnalyzeAnalyzer! Power board must be \"Power board 10XX\". Namespace: %s", n.getNamespace().c_str());*/
     return false;
  }

  // path_ = BASE_NAME/Motors
  if (base_name == "/")
    path_ = base_name + nice_name_;
  else
    path_ = base_name + "/" + nice_name_;

  if (!n.getParam("my_item", my_item_name_))
  {
    /* @todo(anordman):logging RCLCPP_ERROR(get_logger(), "No parameter \"my_item\" found. Unable to initialize MatchNoAnalyzeAnalyzer!");*/
    return false;
  }

  has_initialized_ = true;
  
  return true;
}


bool MatchNoAnalyzeAnalyzer::match(const std::string name)
{
  return has_initialized_ && name == my_item_name_;
}

bool MatchNoAnalyzeAnalyzer::analyze(const std::shared_ptr<StatusItem> item)
{
  /* @todo(anordman):assertion ROS_ASSERT_MSG(get_logger(), item->getName() == my_item_name_, "Asked to analyze item that wasn't mine! My name: %s, item: %s", my_item_name_.c_str(), item->getName().c_str());*/

  return false;
}

vector<std::shared_ptr<diagnostic_msgs::DiagnosticStatus> > MatchNoAnalyzeAnalyzer::report()
{
  vector<std::shared_ptr<diagnostic_msgs::DiagnosticStatus> > output;

  return output;
}
