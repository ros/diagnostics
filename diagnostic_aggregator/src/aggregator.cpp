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
  analyzer_group_(NULL),
  other_analyzer_(NULL),
  base_path_("")
{
  ros::NodeHandle nh = ros::NodeHandle("~");
  nh.param(string("base_path"), base_path_, string(""));
  if (base_path_.size() > 0 && base_path_.find("/") != 0)
    base_path_ = "/" + base_path_;

  nh.param("pub_rate", pub_rate_, pub_rate_);
    
  analyzer_group_ = new AnalyzerGroup();
  
  if (!analyzer_group_->init(base_path_, nh))
  {
    ROS_ERROR("Analyzer group for diagnostic aggregator failed to initialize!");
  }
  

  // Last analyzer handles remaining data
  other_analyzer_ = new OtherAnalyzer();
  other_analyzer_->init(base_path_); // This always returns true

  diag_sub_ = n_.subscribe("/diagnostics", 1000, &Aggregator::diagCallback, this);
  agg_pub_ = n_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics_agg", 1);
  toplevel_state_pub_ = n_.advertise<diagnostic_msgs::DiagnosticStatus>("/diagnostics_toplevel_state", 1);
}

void Aggregator::checkTimestamp(const diagnostic_msgs::DiagnosticArray::ConstPtr& diag_msg)
{
  if (diag_msg->header.stamp.toSec() != 0)
    return;

  string stamp_warn = "No timestamp set for diagnostic message. Message names: ";
  vector<diagnostic_msgs::DiagnosticStatus>::const_iterator it;
  for (it = diag_msg->status.begin(); it != diag_msg->status.end(); ++it)
  {
    if (it != diag_msg->status.begin())
      stamp_warn += ", ";
    stamp_warn += it->name;
  }
  
  if (!ros_warnings_.count(stamp_warn))
  {
    ROS_WARN("%s", stamp_warn.c_str());
    ros_warnings_.insert(stamp_warn);
  }
}

void Aggregator::diagCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& diag_msg)
{
  checkTimestamp(diag_msg);  

  bool analyzed = false;
  for (unsigned int j = 0; j < diag_msg->status.size(); ++j)
  {
    analyzed = false;
    boost::shared_ptr<StatusItem> item(new StatusItem(&diag_msg->status[j]));
    if (analyzer_group_->match(item->getName()))
      analyzed = analyzer_group_->analyze(item);

    if (!analyzed)
      other_analyzer_->analyze(item);
  }
}

Aggregator::~Aggregator() 
{
  if (analyzer_group_) delete analyzer_group_;

  if (other_analyzer_) delete other_analyzer_;
}

void Aggregator::publishData()
{
  diagnostic_msgs::DiagnosticArray diag_array;

  diagnostic_msgs::DiagnosticStatus diag_toplevel_state;
  diag_toplevel_state.name = "toplevel_state";
  diag_toplevel_state.level = -1;
  int min_level = 255;

  vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > processed = analyzer_group_->report();
  for (unsigned int i = 0; i < processed.size(); ++i)
  {
    diag_array.status.push_back(*processed[i]);

    if (processed[i]->level > diag_toplevel_state.level)
      diag_toplevel_state.level = processed[i]->level;
    if (processed[i]->level < min_level)
      min_level = processed[i]->level;
  }
 
  vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > processed_other = other_analyzer_->report();
  for (unsigned int i = 0; i < processed_other.size(); ++i)
  {
    diag_array.status.push_back(*processed_other[i]);

    if (processed_other[i]->level > diag_toplevel_state.level)
      diag_toplevel_state.level = processed_other[i]->level;
    if (processed_other[i]->level < min_level)
      min_level = processed_other[i]->level;
  }

  diag_array.header.stamp = ros::Time::now();

  agg_pub_.publish(diag_array);

  // Top level is error if we have stale items, unless all stale
  if (diag_toplevel_state.level > 2 && min_level <= 2)
    diag_toplevel_state.level = 2;

  toplevel_state_pub_.publish(diag_toplevel_state);
}
