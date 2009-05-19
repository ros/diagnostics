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

#include "ros/node_handle.h"

#include "robot_msgs/DiagnosticMessage.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"

/**

@mainpage

@htmlinclude manifest.html

@b diagnostic_updater.h defines the Updater class, which
simplifies writing of diagnostic publishing code, by allowing a set of
registered callbacks to be published at a fixed rate.

<hr>

@section topics ROS topics

Subscribes to (name/type):
- None

Publishes to (name / type):

@section parameters ROS parameters

Reads the following parameters from the parameter server

- @b "diagnostic_period" : @b [double] period at which diagnostics should be sent in seconds (Default: 1)

**/

/**
\@class Updater Simplifies writing of diagnostic publishing code, by allowing a set of
registered callbacks to be published at a fixed rate.
*/

namespace diagnostic_updater
{
  
typedef boost::function<void(diagnostic_updater::DiagnosticStatusWrapper&)> TaskFunction;

/**
 *
 * The updater_base class is an internally used class that manages a
 * collection of diagnostic updaters. It contains the common functionality
 * used for producing diagnostic updates and for self-checks.
 *
 */

template <class C>
class Updater_base 
{
public:
  Updater_base(C *owner)
  {
    owner_ = owner;
  }
  
  void add(TaskFunction &f)
  {
    tasks_.push_back(f); // @todo Is this acceptable?
  }

  template <class T>
  void add(T *c, void (T::*f)(diagnostic_updater::DiagnosticStatusWrapper&))
  {
    TaskFunction f2 = boost::bind(f, c, _1);
    tasks_.push_back(f2);
  }
  
  void add(void (C::*f)(diagnostic_updater::DiagnosticStatusWrapper&))
  {
    add(owner_, f);
  }

protected:
  std::vector<TaskFunction> tasks_;
  C *owner_;
};

template <class C>
class Updater : public Updater_base<C>
{
public:

  /**
   * Do we really want this constructor?
   */

  /*Updater() : node_handle_()
  {
    setup();
  }*/
  
  Updater(C *owner, ros::NodeHandle h) : Updater_base<C>(owner), node_handle_(h)
  {
    setup();
  }
  
  void update()
  {
    ros::Time now_time = ros::Time::now();
    if (now_time < next_time_) {
      return;
    }

    force_update();
  }

  void force_update()
  {
    next_time_ = ros::Time::now() + ros::Duration().fromSec(period_);
    
    if (node_handle_.ok())
    {
      std::vector<robot_msgs::DiagnosticStatus> status_vec;

      for (std::vector<TaskFunction>::iterator tasks_iter = Updater_base<C>::tasks_.begin();
           tasks_iter != Updater_base<C>::tasks_.end();
           tasks_iter++)
      {
        diagnostic_updater::DiagnosticStatusWrapper status;

        status.name = "None";
        status.level = 2;
        status.message = "No message was set";

        (*tasks_iter)(status);

        if (status.name != "None")
        {
          status.name = node_handle_.getName() + std::string(": ") + status.name;
          status_vec.push_back(status);
        }
      }

      msg_.set_status_vec(status_vec);

      publisher_.publish(msg_);
    }

    node_handle_.param("~diagnostic_period", period_, 1.0);
  }

  double getPeriod()
  {
    return period_;
  }

private:
  void setup()
  {
    publisher_ = node_handle_.advertise<robot_msgs::DiagnosticMessage>("/diagnostics", 1);

    node_handle_.param("~diagnostic_period", period_, 1.0);
    next_time_ = ros::Time::now();
  }

  ros::NodeHandle node_handle_;
  ros::Publisher publisher_;

  ros::Time next_time_;

  double period_;

  robot_msgs::DiagnosticMessage msg_;

};

};

/**
 *
 * Compatibility class to support the nodes that use the old version of the
 * diagnostic_updater. This is deprecated, so avoid using it.
 *
 */

template <class T>
class DiagnosticUpdater : public diagnostic_updater::Updater<T>
{
public:
  DiagnosticUpdater(T *n) : diagnostic_updater::Updater<T>(n, ros::NodeHandle())
  {
    complain();
  }

  DiagnosticUpdater(T *c, ros::Node &n) : diagnostic_updater::Updater<T>(n, ros::NodeHandle())
  {
    complain();
  }

  DiagnosticUpdater(T *c, ros::NodeHandle &h) : diagnostic_updater::Updater<T>(c, h)
  {
    complain();
  }

  void addUpdater(void (T::*f)(robot_msgs::DiagnosticStatus&))
  {
    void (T::*f2)(diagnostic_updater::DiagnosticStatusWrapper&);
    f2 = (typeof f2) f; // @todo Is this acceptable?
    diagnostic_updater::Updater<T>::add(f2);
  }

private:
  void complain()
  {
    //ROS_WARN("DiagnosticUpdater is deprecated, please use diagnostic_updater::Updater instead.");
  }
};

#endif
