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
#include "ros/this_node.h"

#include "diagnostic_msgs/DiagnosticArray.h"
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
  
typedef boost::function<void(DiagnosticStatusWrapper&)> TaskFunction;
typedef boost::function<void(diagnostic_msgs::DiagnosticStatus&)> UnwrappedTaskFunction;

/**
 * DiagnosticTask is an abstract base class for diagnostic tasks.
 * Subclasses will be provided for generating common diagnostic
 * information.
 */

class DiagnosticTask
{
public:
  DiagnosticTask(const std::string name) : name_(name)
  {}
  const std::string &getName()
  {
    return name_;
  }
  virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat) = 0;
  virtual ~DiagnosticTask()
  {}

private:
  const std::string name_;
};

template <class T>
class GenericFunctionDiagnosticTask : public DiagnosticTask
{
public:
  GenericFunctionDiagnosticTask(const std::string &name, boost::function<void(T&)> fn) : 
    DiagnosticTask(name), fn_(fn)
  {}
  
  virtual void run(DiagnosticStatusWrapper &stat)
  {
    fn_(stat);
  }
  
private:
  const std::string name_;
  const TaskFunction fn_;
};

typedef GenericFunctionDiagnosticTask<diagnostic_msgs::DiagnosticStatus> UnwrappedFunctionDiagnosticTask;
typedef GenericFunctionDiagnosticTask<DiagnosticStatusWrapper> FunctionDiagnosticTask;

/**
 * A ComposableDiagnosticTask is a DiagnosticTask that is designed to be
 * composed with other ComposableDiagnosticTask into a single status
 * message using a CombinationDiagnosticTask.
 */

class ComposableDiagnosticTask : public DiagnosticTask
{
public:
  ComposableDiagnosticTask(const std::string name) : DiagnosticTask(name)
  {}
  
  void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    split_run(stat, stat);
  }

  virtual void split_run(diagnostic_updater::DiagnosticStatusWrapper &summary, 
      diagnostic_updater::DiagnosticStatusWrapper &details) = 0;
};

/**
 * The CombinationDiagnosticTask allows multiple ComposableDiagnosticTask instances
 * to be combined into a single DiagnosticStatus. The output of the
 * combination has the max of the status levels, and a concatenation of the
 * non-zero-level messages.
 */

class CombinationDiagnosticTask : public DiagnosticTask
{
public:
  CombinationDiagnosticTask(const std::string name) : DiagnosticTask(name)
  {}

  virtual void run(DiagnosticStatusWrapper &stat)
  {
    DiagnosticStatusWrapper summary;
    stat.summary(0, "");
    
    for (std::vector<ComposableDiagnosticTask *>::iterator i = tasks_.begin();
        i != tasks_.end(); i++)
    {
      (*i)->split_run(summary, stat);
    
      stat.mergeSummary(summary.level, summary.message);
    }
  }
  
  void addTask(ComposableDiagnosticTask *t)
  {
    tasks_.push_back(t);
  }

private:
  std::vector<ComposableDiagnosticTask *> tasks_;
};

/**
 *
 * The @b DiagnosticTaskVector class is abstract base class that manages a
 * collection of diagnostic updaters. It contains the common functionality
 * used for producing diagnostic updates and for self-checks.
 *
 */

class DiagnosticTaskVector
{
protected:
  class DiagnosticTaskInternal
  {
  public:
    DiagnosticTaskInternal(const std::string name, TaskFunction f) :
      name_(name), fn_(f)
    {}

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat) const
    {
      stat.name = name_;
      fn_(stat);
    }

    const std::string &getName() const
    {
      return name_;
    }

  private:
    std::string name_;
    TaskFunction fn_;
  };

  boost::mutex lock_;

  const std::vector<DiagnosticTaskInternal> &getTasks()
  {
    return tasks_;
  }

public:    
  /**
   * \brief Add a DiagnosticTask to the DiagnosticTaskVector
   *
   * \param task The DiagnosticTask to be added. It must remain valid at
   * least until the last time its diagnostic method is called. It need not be
   * valid at the time the DiagnosticTaskVector is destructed.
   */

  void add(const std::string &name, TaskFunction f)
  {
    DiagnosticTaskInternal int_task(name, f);
    addInternal(int_task);
  }

  void add(DiagnosticTask &task)
  {
    TaskFunction f = boost::bind(&DiagnosticTask::run, &task, _1);
    add(task.getName(), f);
  }
  
  template <class T>
  void add(const std::string name, T *c, void (T::*f)(diagnostic_updater::DiagnosticStatusWrapper&))
  {
    DiagnosticTaskInternal int_task(name, boost::bind(f, c, _1));
    addInternal(int_task);
  }
  
private:
  virtual void addedTaskCallback(DiagnosticTaskInternal &)
  {}
  std::vector<DiagnosticTaskInternal> tasks_;
  
protected:
  void addInternal(DiagnosticTaskInternal &task)
  {
    boost::mutex::scoped_lock lock(lock_);
    tasks_.push_back(task); 
    addedTaskCallback(task);
  }
};

class Updater : public DiagnosticTaskVector
{
public:
  Updater(ros::NodeHandle h) : node_handle_(h)
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
      std::vector<diagnostic_msgs::DiagnosticStatus> status_vec;

      boost::mutex::scoped_lock lock(lock_); // Make sure no adds happen while we are processing here.
      const std::vector<DiagnosticTaskInternal> &tasks = getTasks();
      for (std::vector<DiagnosticTaskInternal>::const_iterator iter = tasks.begin();
           iter != tasks.end(); iter++)
      {
        diagnostic_updater::DiagnosticStatusWrapper status;

        status.name = iter->getName();
        status.level = 2;
        status.message = "No message was set";

        iter->run(status);

        status_vec.push_back(status);
      }

      publish(status_vec);
    }

    node_handle_.param("~diagnostic_period", period_, 1.0);
  }

  double getPeriod()
  {
    return period_;
  }

  // Destructor has troble because the node is already shut down.
  /*~Updater()
  {
    // Create a new node handle and publisher because the existing one is 
    // probably shut down at this stage.
    
    ros::NodeHandle newnh; 
    node_handle_ = newnh; 
    publisher_ = node_handle_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
    broadcast(2, "Node shut down"); 
  }*/

  void broadcast(int lvl, const std::string msg)
  {
    std::vector<diagnostic_msgs::DiagnosticStatus> status_vec;
      
    const std::vector<DiagnosticTaskInternal> &tasks = getTasks();
    for (std::vector<DiagnosticTaskInternal>::const_iterator iter = tasks.begin();
        iter != tasks.end(); iter++)
    {
      diagnostic_updater::DiagnosticStatusWrapper status;

      status.name = iter->getName();
      status.summary(lvl, msg);

      status_vec.push_back(status);
    }

    publish(status_vec);
  }

private:
  void publish(diagnostic_msgs::DiagnosticStatus &stat)
  {
    std::vector<diagnostic_msgs::DiagnosticStatus> status_vec;
    status_vec.push_back(stat);
    publish(status_vec);
  }

  void publish(std::vector<diagnostic_msgs::DiagnosticStatus> &status_vec)
  {
    for  (std::vector<diagnostic_msgs::DiagnosticStatus>::iterator 
        iter = status_vec.begin(); iter != status_vec.end(); iter++)
    {
      iter->name = 
        ros::this_node::getName().substr(1) + std::string(": ") + iter->name;
    }
    diagnostic_msgs::DiagnosticArray msg;
    msg.set_status_vec(status_vec);
    publisher_.publish(msg);
  }

  void setup()
  {
    publisher_ = node_handle_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

    node_handle_.param("~diagnostic_period", period_, 1.0);
    next_time_ = ros::Time::now();
  }

  virtual void addedTaskCallback(DiagnosticTaskInternal &task)
  {
    DiagnosticStatusWrapper stat;
    stat.name = task.getName();
    stat.summary(2, "Node starting up");
    publish(stat);
  }

  ros::NodeHandle node_handle_;
  ros::Publisher publisher_;

  ros::Time next_time_;

  double period_;

};

};

/**
 *
 * Compatibility class to support the nodes that use the old version of the
 * diagnostic_updater. This is deprecated, so avoid using it.
 *
 */

/**
 *
 * This class is deprecated. Use diagnostic_updater::Updater instead.
 *
 */

template <class T>
class DiagnosticUpdater : public diagnostic_updater::Updater
{
public:
  ROSCPP_DEPRECATED DiagnosticUpdater(T *n) : diagnostic_updater::Updater(ros::NodeHandle()), owner_(n)
  {
    complain();
  }

  ROSCPP_DEPRECATED DiagnosticUpdater(T *c, ros::Node &n) : diagnostic_updater::Updater(ros::NodeHandle()), owner_(c)
  {
    complain();
  }

  ROSCPP_DEPRECATED DiagnosticUpdater(T *c, ros::NodeHandle &h) : diagnostic_updater::Updater(h), owner_(c)
  {
    complain();
  }

  using diagnostic_updater::Updater::add;

  void addUpdater(void (T::*f)(diagnostic_msgs::DiagnosticStatus&))
  {
    diagnostic_updater::UnwrappedTaskFunction f2 = boost::bind(f, owner_, _1);
    diagnostic_msgs::DiagnosticStatus stat;
    f2(stat); // Get the function to fill out its name.
    boost::shared_ptr<diagnostic_updater::UnwrappedFunctionDiagnosticTask> 
      fcls(new diagnostic_updater::UnwrappedFunctionDiagnosticTask(stat.name, f2));
    tasks_vect_.push_back(fcls);
    add(*fcls);
  }

private:
  void complain()
  {
    //ROS_WARN("DiagnosticUpdater is deprecated, please use diagnostic_updater::Updater instead.");
  }
  
  T *owner_;
  std::vector<boost::shared_ptr<diagnostic_updater::UnwrappedFunctionDiagnosticTask> > tasks_vect_;
};

#endif
