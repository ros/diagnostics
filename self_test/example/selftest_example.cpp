/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include <memory>
#include <stdexcept>

#include "diagnostic_msgs/srv/self_test.hpp"

#include "rclcpp/rclcpp.hpp"

#include "self_test/test_runner.hpp"

class MyNode : public rclcpp::Node
{
private:
  //  self_test::TestRunner is the handles sequencing driver self-tests.
  self_test::TestRunner self_test_;
  //  A value showing statefulness of tests
  double some_val_;

public:
  MyNode()
  : rclcpp::Node("self_test_example_node"),
    self_test_(
      get_node_base_interface(), get_node_services_interface(), get_node_logging_interface())
  {
    //  If any setup work needs to be done before running the tests,
    //  a pretest can be defined. It is just like any other test, but
    //  doesn't actually do any testing.
    self_test_.add("Pretest", this, &MyNode::pretest);

    //  Tests added will be run in the order in which they are added. Each
    //  test has a name that will be automatically be filled in the
    //  DiagnosticStatus message.
    //
    //  NOTE: self_test::TestRunner inherits its add() methods from
    //  diagnostic_updater::DiagnosticTaskVector. You will have to refer to
    //  the diagnostic_updater doxygen documentation to find them:
    self_test_.add("ID Lookup", this, &MyNode::test1);
    self_test_.add("Exception generating test", this, &MyNode::test2);
    self_test_.add("Value generating test", this, &MyNode::test3);
    self_test_.add("Value testing test", this, &MyNode::test4);
    self_test_.add("Value testing test2", this, &MyNode::test4);

    //  You can remove a test by using its name (documented in
    //  diagnostic_updater).
    if (!self_test_.removeByName("Value testing test2")) {
      //  If any cleanup work needs to be done after running the tests,
      //  a posttest can be defined. It is just like any other test, but
      //  doesn't actually do any testing.
      self_test_.add("Posttest", this, &MyNode::pretest);
    }
  }

  void pretest(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Pretest completed successfully.");

    some_val_ = 1.0;
  }

  //  All tests take a reference to a DiagnosticStatusWrapper message which they should populate
  //  The default values are status.level = 2 (ERROR), and status.message = "No message was set"
  //  The status.name is automatically set to the name that was passed to add.
  //  A DiagnosticStatusWrapper is used instead of a DiagnosticStatus
  //  because it provides useful convenience methods.
  void test1(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    //  Look up ID here
    char ID[] = "12345";
    bool lookup_successful = true;

    if (lookup_successful) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "ID Lookup successful");

      //  One of the tests should call setID() to fill the hardware
      //  identifier in the self-test output. In cases that do not involve
      //  hardware, or for which there is no hardware identifier, an
      //  identifier of "none" should be used.
      //  For hardware devices that have a hardware identifier, the setID()
      //  method should be called
      self_test_.setID(ID);

    } else {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "ID Lookup failed");
    }
  }

  //  Tests do not necessarily need to catch their exceptions.
  void test2(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    //  Note, we start setting our status to success.  Since our
    //  exception is not caught, however, the SelfTest class will
    //  change level to ERROR.  This wouldn't be common practice And I
    //  would instead recommend not changing the status to success
    //  until after the exception might be generated.

    status.level = 0;

    //  Here's where we would report success if we'd made it past
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "We made it past the exception throwing statement.");
  }

  //  The state of the node can be changed as the tests are operating
  void test3(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    //  Do something that changes the state of the node
    some_val_ += 41.0;

    status.add("some value", some_val_);
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "We successfully changed the value.");
  }

  void test4(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    if (some_val_ == 42.0) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "We observed the change in value");
    } else {
      status.summaryf(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "We failed to observe the change in value, it is currently %f.", some_val_);
    }
  }

  void posttest(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Posttest completed successfully.");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto n = std::make_shared<MyNode>();
  rclcpp::spin(n);

  rclcpp::shutdown();
  return 0;
}
