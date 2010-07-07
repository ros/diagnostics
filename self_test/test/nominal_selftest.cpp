/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>
#include "diagnostic_msgs/SelfTest.h"
#include "self_test/self_test.h"
#include <stdexcept>

/*
 *\author Kevin Watts
 *\brief Returns nominal self-test values
 */

class MyNode 
{
public:

  // self_test::TestRunner is the handles sequencing driver self-tests.
  self_test::TestRunner self_test_;

  // A value showing statefulness of tests
  double some_val;

  ros::NodeHandle nh_;

  MyNode() : self_test_()
  {
    self_test_.add("Pretest", this, &MyNode::pretest );

    self_test_.add("ID Lookup",                 this, &MyNode::test1);
    self_test_.add("Exception generating test", this, &MyNode::test2);
    self_test_.add("Value generating test",     this, &MyNode::test3);
    self_test_.add("Value testing test",        this, &MyNode::test4);

    self_test_.add("Posttest", this, &MyNode::pretest );
  }

  void pretest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    ROS_INFO("Doing preparation stuff before we run our test.\n");
    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Pretest completed successfully.");
    
    some_val = 1.0;
  }

  void test1(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    // Look up ID here
    char ID[] = "12345";
    bool lookup_successful = true;

    if (lookup_successful)
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "ID Lookup successful");
      
      self_test_.setID(ID);

    } else {
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "ID Lookup failed");
    }
  }

  // Tests do not necessarily need to catch their exceptions.
  void test2(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.level = 0;

    // Here's where we would report success if we'd made it past
    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "We made it past the exception throwing statement.");
  }

  void test3(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    some_val += 41.0;

    status.add("some value", some_val);
    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "We successfully changed the value.");
  }

  void test4(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    if (some_val == 42.0)
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "We observed the change in value");
    } 
    else
    {
      status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "We failed to observe the change in value, it is currently %f.", some_val);
    }
  }

  void posttest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    ROS_INFO("Doing cleanup stuff after we run our test.\n");
    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Posttest completed successfully.");
  }

  bool spin()
  {
    while (nh_.ok())
    {
      ros::Duration(1).sleep();
      
      self_test_.checkTest();
    }
    return true;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_node");

  MyNode n;

  n.spin();

  return(0);
}
