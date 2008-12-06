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
#include "rosthread/condition.h"

#include "robot_msgs/DiagnosticStatus.h"
#include "robot_srvs/SelfTest.h"



template <class T>
class SelfTest
{
private:

  T* node_;
  ros::thread::condition testing_condition;
  ros::thread::condition done_testing_condition;

  std::string id_;

  std::vector<void (T::*)(robot_msgs::DiagnosticStatus&)> test_fncs;

  void (T::*pretest_)();
  void (T::*posttest_)();

  int count;

  bool waiting;
  bool ready;
  bool done;

public:

  SelfTest(T* node) : node_(node), pretest_(NULL), posttest_(NULL)
  {
    node_->advertise_service("~self_test", &SelfTest::doTest, this);
    count = 0;
    waiting = false;
    ready   = false;
    done    = false;
  }

  void addTest(void (T::*f)(robot_msgs::DiagnosticStatus&))
  {
    test_fncs.push_back(f);
  }

  void setPretest(void (T::*f)())
  {
    pretest_ = f;
  }

  void setPosttest(void (T::*f)())
  {
    posttest_ = f;
  }

  void checkTest()
  {
    testing_condition.lock();
    if (waiting)
    {
      ready = true;
      testing_condition.signal();
      testing_condition.unlock();

      done_testing_condition.lock();
      done = false;
      while (!done)
        done_testing_condition.wait();
      done_testing_condition.unlock();
    } else {
      testing_condition.unlock();
    }
    sched_yield();
  }

  void setID(std::string id)
  {
    id_ = id;
  }

  bool doTest(robot_srvs::SelfTest::request &req,
                robot_srvs::SelfTest::response &res)
  {
    testing_condition.lock();

    waiting = true;
    ready   = false;

    while (!ready)
    {
      if (!testing_condition.timed_wait(ros::Duration().fromSec(5.0)))
      {
        printf("Timed out waiting to run self test.\n");
        waiting = false;
        testing_condition.unlock();
        return false;
      }
    }

    testing_condition.unlock();

    bool retval = false;

    printf("Begining test.\n");

    if (node_->ok())
    {

      id_ = "";

      printf("Entering self test.  Other operation should be suspended\n");

      if (pretest_ != NULL)
        (*node_.*pretest_)();
    
      printf("Completed pretest\n");

      std::vector<robot_msgs::DiagnosticStatus> status_vec;

      for (typename std::vector<void (T::*)(robot_msgs::DiagnosticStatus&)>::iterator test_fncs_iter = test_fncs.begin();
           test_fncs_iter != test_fncs.end();
           test_fncs_iter++)
      {
        robot_msgs::DiagnosticStatus status;

        status.name = "None";
        status.level = 2;
        status.message = "No message was set";

        try {

          (*node_.*(*test_fncs_iter))(status);

        } catch (std::exception& e)
        {
          status.level = 2;
          status.message = std::string("Uncaught exception: ") + e.what();
        }

        status_vec.push_back(status);
      }

      waiting = false;
      testing_condition.unlock();

      //One of the test calls should use setID
      res.id = id_;

      res.passed = true;
      for (std::vector<robot_msgs::DiagnosticStatus>::iterator status_iter = status_vec.begin();
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
        (*node_.*posttest_)();

      printf("Self test completed\n");

      retval = true;

    }

    done_testing_condition.lock();
    done = true;
    done_testing_condition.signal();
    done_testing_condition.unlock();

    return retval;

  }
};

#endif
