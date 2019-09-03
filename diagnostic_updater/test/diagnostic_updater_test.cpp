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

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/update_functions.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

class TestClass
{
public:
  void unwrapped(diagnostic_msgs::msg::DiagnosticStatus & s)
  {
    (void) s;
  }

  void wrapped(diagnostic_updater::DiagnosticStatusWrapper & s)
  {
    (void) s;
  }
};

class classFunction : public diagnostic_updater::DiagnosticTask
{
public:
  classFunction()
  : DiagnosticTask("classFunction") {}

  void run(diagnostic_updater::DiagnosticStatusWrapper & s)
  {
    s.summary(0, "Test is running");
    s.addf("Value", "%f", 5);
    s.add("String", "Toto");
    s.add("Floating", 5.55);
    s.add("Integer", 5);
    s.addf("Formatted %s %i", "Hello", 5);
    s.add("Bool", true);
  }
};

TEST(DiagnosticUpdater, testDiagnosticUpdater) {
  rclcpp::init(0, nullptr);

  TestClass c;

  auto node = std::make_shared<rclcpp::Node>("test_diagnostics_updater");
  diagnostic_updater::Updater updater(node);

  updater.add("wrapped", &c, &TestClass::wrapped);

  classFunction cf;
  updater.add(cf);

  rclcpp::shutdown();
  SUCCEED();
}

TEST(DiagnosticUpdater, testLifecycleDiagnosticUpdater) {
  rclcpp::init(0, nullptr);

  TestClass c;

  auto node =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_lifeycycle_diagnostics_updater");
  diagnostic_updater::Updater updater(node);

  updater.add("wrapped", &c, &TestClass::wrapped);

  classFunction cf;
  updater.add(cf);

  rclcpp::shutdown();
  SUCCEED();
}

class DiagnosticWrapperNode : public rclcpp::Node
{
public:
  DiagnosticWrapperNode()
  : Node("DiagnosticUpdaterTestNode"),
    updater_(this)
  {}

private:
  diagnostic_updater::Updater updater_;
};

TEST(DiagnosticUpdater, testUpdaterAsNodeClassMember) {
  rclcpp::init(0, nullptr);
  DiagnosticWrapperNode node;
  rclcpp::shutdown();
  SUCCEED();
}

TEST(DiagnosticUpdater, testDiagnosticStatusWrapperKeyValuePairs) {
  diagnostic_updater::DiagnosticStatusWrapper stat;

  const char * message = "dummy";
  int level = 1;
  stat.summary(level, message);
  EXPECT_STREQ(message, stat.message.c_str()) <<
    "DiagnosticStatusWrapper::summary failed to set message";
  EXPECT_EQ(level, stat.level) <<
    "DiagnosticStatusWrapper::summary failed to set level";

  stat.addf("toto", "%.1f", 5.0);
  stat.add("baba", 5);
  stat.addf("foo", "%05i", 27);

  stat.add("bool", true);
  stat.add("bool2", false);

  EXPECT_STREQ("5.0", stat.values[0].value.c_str()) <<
    "Bad value, adding a value with addf";
  EXPECT_STREQ("5", stat.values[1].value.c_str()) <<
    "Bad value, adding a string with add";
  EXPECT_STREQ("00027", stat.values[2].value.c_str()) <<
    "Bad value, adding a string with addf";
  EXPECT_STREQ("toto", stat.values[0].key.c_str()) <<
    "Bad label, adding a value with add";
  EXPECT_STREQ("baba", stat.values[1].key.c_str()) <<
    "Bad label, adding a string with add";
  EXPECT_STREQ("foo", stat.values[2].key.c_str()) <<
    "Bad label, adding a string with addf";

  EXPECT_STREQ("bool", stat.values[3].key.c_str()) <<
    "Bad label, adding a true bool key with add";
  EXPECT_STREQ("True", stat.values[3].value.c_str()) <<
    "Bad label, adding a true bool with add";

  EXPECT_STREQ("bool2", stat.values[4].key.c_str()) <<
    "Bad label, adding a false bool key with add";
  EXPECT_STREQ("False", stat.values[4].value.c_str()) <<
    "Bad label, adding a false bool with add";
}

TEST(DiagnosticUpdater, testDiagnosticStatusWrapperMergeSummary) {
  diagnostic_updater::DiagnosticStatusWrapper stat;

  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Old");
  stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::OK, "New");
  EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::OK, stat.level) <<
    "Bad level, merging levels (OK,OK)";
  EXPECT_STREQ("Old", stat.message.c_str()) <<
    "Bad summary, merging levels (OK,OK)";

  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Old");
  stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "New");
  EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::WARN, stat.level) <<
    "Bad level, merging levels (OK,WARN)";
  EXPECT_STREQ("New", stat.message.c_str()) <<
    "Bad summary, merging levels (OK,WARN)";

  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Old");
  stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "New");
  EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::WARN, stat.level) <<
    "Bad level, merging levels (WARN,WARN)";
  EXPECT_STREQ("Old; New", stat.message.c_str()) <<
    "Bad summary, merging levels (WARN,WARN)";

  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Old");
  stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "New");
  EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::ERROR, stat.level) <<
    "Bad level, merging levels (WARN,ERROR)";
  EXPECT_STREQ("Old; New", stat.message.c_str()) <<
    "Bad summary, merging levels (WARN,ERROR)";
}

TEST(DiagnosticUpdater, testFrequencyStatus) {
  double minFreq = 10;
  double maxFreq = 20;

  diagnostic_updater::FrequencyStatus fs(
    diagnostic_updater::FrequencyStatusParam(&minFreq, &maxFreq, 0.5, 2));

  diagnostic_updater::DiagnosticStatusWrapper stat[5];
  fs.tick();
  std::this_thread::sleep_for(20ms);
  fs.run(stat[0]);  // Should be too fast, 20 ms for 1 tick, lower limit should
                    // be 33ms.
  std::this_thread::sleep_for(50ms);
  fs.tick();
  fs.run(stat[1]);  // Should be good, 70 ms for 2 ticks, lower limit should be
                    // 66 ms.
  std::this_thread::sleep_for(300ms);
  fs.tick();
  fs.run(stat[2]);  // Should be good, 350 ms for 2 ticks, upper limit should be
                    // 400 ms.
  std::this_thread::sleep_for(150ms);
  fs.tick();
  fs.run(stat[3]);  // Should be too slow, 450 ms for 2 ticks, upper limit should
                    // be 400 ms.
  fs.clear();
  fs.run(stat[4]);  // Should be good, just cleared it.

  EXPECT_EQ(1, stat[0].level) << "max frequency exceeded but not reported";
  EXPECT_EQ(0, stat[1].level) << "within max frequency but reported error";
  EXPECT_EQ(0, stat[2].level) << "within min frequency but reported error";
  EXPECT_EQ(1, stat[3].level) << "min frequency exceeded but not reported";
  EXPECT_EQ(2, stat[4].level) << "freshly cleared should fail";
  EXPECT_STREQ("", stat[0].name.c_str()) <<
    "Name should not be set by FrequencyStatus";
  EXPECT_STREQ("Frequency Status", fs.getName().c_str()) <<
    "Name should be \"Frequency Status\"";
}

TEST(DiagnosticUpdater, testTimeStampStatus) {
  diagnostic_updater::TimeStampStatus ts(
    diagnostic_updater::DefaultTimeStampStatusParam);

  diagnostic_updater::DiagnosticStatusWrapper stat[5];
  ts.run(stat[0]);
  ts.tick(rclcpp::Clock().now().seconds() + 2);
  ts.run(stat[1]);
  ts.tick(rclcpp::Clock().now());
  ts.run(stat[2]);
  ts.tick(rclcpp::Clock().now().seconds() - 4);
  ts.run(stat[3]);
  ts.tick(rclcpp::Clock().now().seconds() - 6);
  ts.run(stat[4]);

  EXPECT_EQ(1, stat[0].level) << "no data should return a warning";
  EXPECT_EQ(2, stat[1].level) << "too far future not reported";
  EXPECT_EQ(0, stat[2].level) << "now not accepted";
  EXPECT_EQ(0, stat[3].level) << "4 seconds ago not accepted";
  EXPECT_EQ(2, stat[4].level) << "too far past not reported";
  EXPECT_STREQ("", stat[0].name.c_str()) <<
    "Name should not be set by TimeStapmStatus";
  EXPECT_STREQ("Timestamp Status", ts.getName().c_str()) <<
    "Name should be \"Timestamp Status\"";
}
