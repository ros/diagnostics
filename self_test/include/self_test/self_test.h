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

#include <boost/thread.hpp>
#include <ros/callback_queue.h>

#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/SelfTest.h"
#include "diagnostic_updater/diagnostic_updater.h"

namespace self_test
{

  using namespace diagnostic_updater;

  /**
   * \brief Class to facilitate the creation of component self-tests.
   *
   * The self_test::TestRunner class advertises the "self_test" service, and
   * maintains a list of pretests and tests. When "self_test" is invoked,
   * TestRunner waits for a suitable time to interrupt the node and run the
   * tests. Results from the tests are collected and returned to the caller.
   */

  class TestRunner : public DiagnosticTaskVector
  {        
    private:
      ros::ServiceServer service_server_;
      ros::CallbackQueue self_test_queue_;
      ros::NodeHandle node_handle_;
      std::string id_;

      bool verbose;

    public:
      using DiagnosticTaskVector::add;

      /**
       * \brief Constructs a dispatcher.
       *
       * \param owner Class that owns this dispatcher. This is used as the
       * default class for tests that are member-functions.
       *
       * \param h NodeHandle from which to work. (Currently unused?)
       */

      TestRunner(ros::NodeHandle h = ros::NodeHandle()) : 
        node_handle_(h)
    {
      ROS_DEBUG("Advertising self_test");
      ros::NodeHandle private_node_handle_("~");
      private_node_handle_.setCallbackQueue(&self_test_queue_);
      service_server_ = private_node_handle_.advertiseService("self_test", &TestRunner::doTest, this);
      verbose = true;
    }

      /**
       * \brief Check if a self-test is pending. If so, start it and wait for it
       * to complete.
       */

      void checkTest()
      {
        self_test_queue_.callAvailable(ros::WallDuration(0));
      }

      /**
       * \brief Sets the ID of the part being tested.
       *
       * This method is expected to be called by one of the tests during the
       * self-test.
       *
       * \param id : String that identifies the piece of hardware being tested.
       */

      void setID(std::string id)
      {
        id_ = id;
      }

    protected:
      /**
       * \brief Internal use.
       *
       * @todo remove once the deprecated methods are removed.
       */
      
      virtual void doPretest()
      {
      }

      /**
       * \brief Internal use.
       *
       * @todo remove once the deprecated methods are removed.
       */
      
      virtual void doPosttest()
      {
      }

    private:
      /**
       * The service callback for the "self-test" service.
       */
      bool doTest(diagnostic_msgs::SelfTest::Request &req,
          diagnostic_msgs::SelfTest::Response &res)
      {
        bool retval = false;

        ROS_INFO("Begining test.\n");

        if (node_handle_.ok())
        {

          id_ = "";

          ROS_INFO("Entering self test.  Other operation should be suspended\n");

          doPretest();
          
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
              if (verbose)
                ROS_WARN("Non-zero self-test test status. Name: '%s', status %i: '%s'", status_iter->name.c_str(), status_iter->level, status_iter->message.c_str());
            }
          }

          res.set_status_vec(status_vec);

          doPosttest();
          
          retval = true;
        }

        return retval;

      }
  };

  /**
   * \brief Deprecated, use TestRunner instead.
   */

  template <class T>
    class Dispatcher : public TestRunner
  {  
    protected: 
      T *owner_;
      void (T::*pretest_)(void);
      void (T::*posttest_)(void);

      virtual void doPretest()
      {
          if (pretest_)
          {
            (owner_->*pretest_)();
            ROS_WARN("self_test pretests are obsolete. Please just use a normal test instead of a pretest.");
            ROS_INFO("Completed pretest");
          }
      }

      virtual void doPosttest()
      {
          if (posttest_)
          {
            (owner_->*posttest_)();
            ROS_WARN("self_test posttests are obsolete. Please just use a normal test instead of a posttest.");
            ROS_INFO("Completed posttest");
          }
      }

    public:
      /**
       * \brief Constructs a legacy dispatcher.
       *
       * \param owner Class that owns this dispatcher. This is used as the
       * default class for tests that are member-functions.
       *
       * \param h NodeHandle from which to work. (Currently unused?)
       */

      ROSCPP_DEPRECATED Dispatcher(T *owner, ros::NodeHandle h) : TestRunner(h), owner_(owner)
      {}

      /**
       * \brief Sets a method to call before the tests.
       *
       * \param f : Method of the owner class to call.
       */

      ROSCPP_DEPRECATED void setPretest(void (T::*f)())
      {
        pretest_ = f;
      }

      /**
       * \brief Sets a method to call after the tests.
       *
       * \param f : Method of the owner class to call.
       */

      ROSCPP_DEPRECATED void setPosttest(void (T::*f)())
      {
        posttest_ = f;
      }

      /**
       * \brief Add a test that is a method of the owner class.
       *
       * \param name : Name of the test. Will be filled into the
       * DiagnosticStatusWrapper automatically.
       *
       * \param f : Method of the owner class that fills out the
       * DiagnosticStatusWrapper.
       */

      using TestRunner::add;
      
      template<class S>
        void add(const std::string name, void (S::*f)(diagnostic_updater::DiagnosticStatusWrapper&))
        {
          DiagnosticTaskInternal int_task(name, boost::bind(f, owner_, _1));
          addInternal(int_task);
        }
  };

};

/**
 *
 * This class is deprecated. Use self_test::TestRunner instead.
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
      //ROS_WARN("SelfTest is deprecated, please use self_test::TestRunner instead.");
    }

  private:
    std::vector<boost::shared_ptr<diagnostic_updater::UnwrappedFunctionDiagnosticTask> > tests_vect_;
};

#endif
