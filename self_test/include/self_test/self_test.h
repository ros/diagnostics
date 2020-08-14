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
   *
   * self_test::TestRunner's is a derived class from <a
   * href="../../diagnostic_updater/html/classdiagnostic__updater_1_1DiagnosticTaskVector.html">diagnostic_updater::DiagnosticTaskVector</a>.
   * For documentation of the add() methods you will have to refer to the
   * base class's documentation.
   */

  class TestRunner : public DiagnosticTaskVector
  {        
    private:
      ros::ServiceServer service_server_;
      ros::CallbackQueue self_test_queue_;
      ros::NodeHandle node_handle_;
      ros::NodeHandle private_node_handle_;
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
      TestRunner(ros::NodeHandle h = ros::NodeHandle(), ros::NodeHandle ph = ros::NodeHandle("~")) : 
        node_handle_(h),
        private_node_handle_(ph)
    {
      ROS_DEBUG("Advertising self_test");
      ros::AdvertiseServiceOptions ops;//use options so that we can set callback queue directly
      ops.init<diagnostic_msgs::SelfTest::Request, diagnostic_msgs::SelfTest::Response>("self_test", boost::bind(&TestRunner::doTest, this, _1, _2));
      ops.callback_queue = &self_test_queue_;
      service_server_ = private_node_handle_.advertiseService(ops);
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

    private:
      /**
       * The service callback for the "self-test" service.
       */
      bool doTest(diagnostic_msgs::SelfTest::Request &req,
          diagnostic_msgs::SelfTest::Response &res)
      {
        bool retval = false;
        bool ignore_set_id_warn = false;

        if (node_handle_.ok())
        {
          const std::string unspecified_id("unspecified");

          ROS_INFO("Entering self-test.");

          std::vector<diagnostic_msgs::DiagnosticStatus> status_vec;

          const std::vector<DiagnosticTaskInternal> &tasks = getTasks();
          for (std::vector<DiagnosticTaskInternal>::const_iterator iter = tasks.begin();
              iter != tasks.end(); iter++)
          {
            if (ros::isShuttingDown())
            {
              ROS_ERROR("ROS has shut down. Exiting.");
              return false;
            }
            
            diagnostic_updater::DiagnosticStatusWrapper status;

            status.level = 2;
            status.message = "No message was set";

            try {
              ROS_INFO("Starting test: %s", iter->getName().c_str());
              iter->run(status);

            } catch (std::exception& e)
            {
              status.level = 2;
              status.message = std::string("Uncaught exception: ") + e.what();
              ignore_set_id_warn = true;
            }

            if (status.level >= 1)
              if (verbose)
                ROS_WARN("Non-zero self-test test status. Name: '%s', status %i: '%s'", status.name.c_str(), status.level, status.message.c_str());
            
            status_vec.push_back(status);
          }

          if (!ignore_set_id_warn && id_.empty())
            ROS_WARN("setID was not called by any self-test. The node author should be notified. If there is no suitable ID for this node, an ID of 'none' should be used.");

          //One of the test calls should use setID
          res.id = id_;

          res.passed = true;
          for (std::vector<diagnostic_msgs::DiagnosticStatus>::iterator status_iter = status_vec.begin();
              status_iter != status_vec.end();
              status_iter++)
          {
            if (status_iter->level >= 2)
              res.passed = false;
          }

          if (res.passed && id_ == unspecified_id)
            ROS_WARN("Self-test passed, but setID was not called. This is a bug in the driver. Please report it.");

          res.status = status_vec;

          retval = true;
          
          ROS_INFO("Self-test complete.");
        }

        return retval;

      }
  };
}

#endif
