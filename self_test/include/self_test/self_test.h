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

#ifndef SELFTEST_HH
#define SELFTEST_HH

#include <stdexcept>
#include <vector>
#include <string>

#include "ros/node.h"
#include <boost/thread.hpp>

#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/SelfTest.h"
#include "diagnostic_updater/diagnostic_updater.h"

namespace self_test
{

using namespace diagnostic_updater;

template <class T>
class Dispatcher : public DiagnosticTaskVector
{        
private:

  boost::mutex testing_mutex;
  boost::condition_variable testing_condition;
  boost::mutex done_testing_mutex;
  boost::condition_variable done_testing_condition;

  ros::NodeHandle node_handle_;
  
  std::string id_;

protected: /// @todo Get rid of this when we get rid of the legacy interface.
  T *owner_;

private:
  void (T::*pretest_)();
  void (T::*posttest_)();

  int count;

  bool waiting;
  bool ready;
  bool done;

  ros::ServiceServer service_server_;

public:

  using DiagnosticTaskVector::add;

  Dispatcher(T *owner, ros::NodeHandle h) : 
    node_handle_(h), owner_(owner), pretest_(NULL), posttest_(NULL)
  {
    ROS_DEBUG("Advertising self_test");
    service_server_ = node_handle_.advertiseService("~self_test", &Dispatcher::doTest, this);
    count = 0;
    waiting = false;
    ready   = false;
    done    = false;
  }

  void setPretest(void (T::*f)())
  {
    pretest_ = f;
  }

  void setPosttest(void (T::*f)())
  {
    posttest_ = f;
  }

  template<class S>
  void add(const std::string name, void (S::*f)(diagnostic_updater::DiagnosticStatusWrapper&))
  {
    DiagnosticTaskInternal int_task(name, boost::bind(f, owner_, _1));
    addInternal(int_task);
  }
  
  void checkTest()
  {
    bool local_waiting = false;
    {
      boost::mutex::scoped_lock lock(testing_mutex);
      local_waiting = waiting;
      ready = true;

      testing_condition.notify_all();
    }

    if (local_waiting)
    {
      boost::mutex::scoped_lock lock(done_testing_mutex);

      done = false;
      while (!done)
      {
        done_testing_condition.wait(lock);
      }
    }

    boost::this_thread::yield();
  }

  void setID(std::string id)
  {
    id_ = id;
  }

  bool doTest(diagnostic_msgs::SelfTest::Request &req,
                diagnostic_msgs::SelfTest::Response &res)
  {
    {
      boost::mutex::scoped_lock lock(testing_mutex);

      waiting = true;
      ready   = false;

      while (!ready)
      {
        if (!testing_condition.timed_wait(lock, boost::get_system_time() + boost::posix_time::seconds(10)))
        {
          diagnostic_updater::DiagnosticStatusWrapper status;
          status.name = "Wait for Node Ready";
          status.level = 2;
          status.message = "Timed out waiting to run self test.";
          ROS_ERROR("Timed out waiting to run self test.\n");
	  res.passed = false;
          res.status.push_back(status);
          waiting = false;
          return true;
        }
      }
    }

    bool retval = false;

    ROS_INFO("Begining test.\n");

    if (node_handle_.ok())
    {

      id_ = "";

      ROS_INFO("Entering self test.  Other operation should be suspended\n");

      if (pretest_ != NULL)
      {
        (owner_->*pretest_)();
      }

      ROS_INFO("Completed pretest");

      std::vector<diagnostic_msgs::DiagnosticStatus> status_vec;

      const std::vector<DiagnosticTaskInternal> &tasks = getTasks();
      for (std::vector<DiagnosticTaskInternal>::const_iterator iter = tasks.begin();
          iter != tasks.end(); iter++)
      {
        diagnostic_updater::DiagnosticStatusWrapper status;

        status.name = "None";
        status.level = 2;
        status.message = "No message was set";

        try {

          iter->run(status);

        } catch (std::exception& e)
        {
          status.level = 2;
          status.message = std::string("Uncaught exception: ") + e.what();
        }

        status_vec.push_back(status);
      }

      {
        boost::mutex::scoped_lock lock(testing_mutex);
        waiting = false;
      }

      //One of the test calls should use setID
      res.id = id_;

      res.passed = true;
      for (std::vector<diagnostic_msgs::DiagnosticStatus>::iterator status_iter = status_vec.begin();
           status_iter != status_vec.end();
           status_iter++)
      {
        if (status_iter->level >= 2)
        {
          res.passed = false;
        }
      }

      res.set_status_vec(status_vec);

      if (posttest_ != NULL)
        (owner_->*posttest_)();

      ROS_INFO("Self test completed");

      retval = true;

    }

    {
      boost::mutex::scoped_lock lock(done_testing_mutex);
      done = true;

      done_testing_condition.notify_all();
    }

    return retval;

  }
};

};

/**
 *
 * This class is deprecated. Use self_test::Dispatcher instead.
 *
 */

template <class T>
class SelfTest : public self_test::Dispatcher<T>
{
public:
  ROSCPP_DEPRECATED SelfTest(T *n) : self_test::Dispatcher<T>(n, ros::NodeHandle())
  {
  }

  void addTest(void (T::*f)(diagnostic_updater::DiagnosticStatusWrapper&))
  {
    self_test::Dispatcher<T>::add("", f);
  }

  void addTest(void (T::*f)(diagnostic_msgs::DiagnosticStatus&))
  {
    diagnostic_updater::UnwrappedTaskFunction f2 = boost::bind(f, self_test::Dispatcher<T>::owner_, _1);
    boost::shared_ptr<diagnostic_updater::UnwrappedFunctionDiagnosticTask> 
      fcls(new diagnostic_updater::UnwrappedFunctionDiagnosticTask("", f2));
    tests_vect_.push_back(fcls);
    self_test::DiagnosticTaskVector::add(*fcls);
  }
  
  void complain()
  {
    //ROS_WARN("SelfTest is deprecated, please use self_test::Dispatcher instead.");
  }

private:
  std::vector<boost::shared_ptr<diagnostic_updater::UnwrappedFunctionDiagnosticTask> > tests_vect_;
};

#endif
