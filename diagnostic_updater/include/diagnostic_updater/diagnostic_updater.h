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

#ifndef DIAGNOSTICUPDATER_HH
#define DIAGNOSTICUPDATER_HH

#include <stdexcept>
#include <vector>
#include <string>

#include "ros/node.h"

#include "robot_msgs/DiagnosticMessage.h"


template <class T>
class DiagnosticUpdater
{
private:

  T* node_;

  std::vector<void (T::*)(robot_msgs::DiagnosticStatus&)> status_fncs_;

  ros::Time next_time_;

  double period_;

  robot_msgs::DiagnosticMessage msg_;

public:

  DiagnosticUpdater(T* node) : node_(node)
  {
    ((ros::node*)(node_))->advertise<robot_msgs::DiagnosticMessage>("/diagnostics", 1);

    node_->param("~diagnostic_period", period_, 1.0);
    next_time_ = ros::Time::now() + ros::Duration(period_);
  }

  void addUpdater(void (T::*f)(robot_msgs::DiagnosticStatus&))
  {
    status_fncs_.push_back(f);
  }

  void update()
  {
    ros::Time now_time = ros::Time::now();
    if (now_time < next_time_) {
      return;
    }

    msg_.set_status_size(status_fncs_.size());

    if (node_->ok())
    {
      std::vector<robot_msgs::DiagnosticStatus> status_vec;

      for (typename std::vector<void (T::*)(robot_msgs::DiagnosticStatus&)>::iterator status_fncs_iter = status_fncs_.begin();
           status_fncs_iter != status_fncs_.end();
           status_fncs_iter++)
      {
        robot_msgs::DiagnosticStatus status;

        status.name = "None";
        status.level = 2;
        status.message = "No message was set";

        (*node_.*(*status_fncs_iter))(status);

        status_vec.push_back(status);
      }

      msg_.set_status_vec(status_vec);

      node_->publish("/diagnostics", msg_);
    }

    node_->param("~diagnostic_period", period_, 1.0);
    next_time_ = ros::Time::now() + ros::Duration(period_);
  }

  double getPeriod()
  {
    return period_;
  }
};

#endif
