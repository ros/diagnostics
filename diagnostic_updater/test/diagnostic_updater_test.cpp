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
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#ifndef _WIN32
#include <unistd.h>
#endif

using namespace diagnostic_updater;

class TestClass 
{
public: 
  void unwrapped(diagnostic_msgs::DiagnosticStatus &s)
  {
  }

  void wrapped(DiagnosticStatusWrapper &s)
  {
  }
};
                                   
TEST(DiagnosticUpdater, testDiagnosticUpdater)
{
  class classFunction : public DiagnosticTask
  {
  public:
    classFunction() : DiagnosticTask("classFunction")
    {}

    void run(DiagnosticStatusWrapper &s) 
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
  
  TestClass c;
  ros::NodeHandle nh;
  
  Updater updater;
  
  updater.add("wrapped", &c, &TestClass::wrapped);
  
  classFunction cf;
  updater.add(cf);
}

TEST(DiagnosticUpdater, testDiagnosticStatusWrapperKeyValuePairs)
{
  DiagnosticStatusWrapper stat;
  
  const char *message = "dummy";
  int level = 1;
  stat.summary(level, message);
  EXPECT_STREQ(message, stat.message.c_str()) << "DiagnosticStatusWrapper::summary failed to set message";
  EXPECT_EQ(level, stat.level) << "DiagnosticStatusWrapper::summary failed to set level";

  stat.addf("toto", "%.1f", 5.0);
  stat.add("baba", 5);
  stat.addf("foo", "%05i", 27);

  stat.add("bool", true);
  stat.add("bool2", false);
  
  EXPECT_STREQ("5.0", stat.values[0].value.c_str()) << "Bad value, adding a value with addf";
  EXPECT_STREQ("5", stat.values[1].value.c_str()) << "Bad value, adding a string with add";
  EXPECT_STREQ("00027", stat.values[2].value.c_str()) << "Bad value, adding a string with addf";
  EXPECT_STREQ("toto", stat.values[0].key.c_str()) << "Bad label, adding a value with add";
  EXPECT_STREQ("baba", stat.values[1].key.c_str()) << "Bad label, adding a string with add";
  EXPECT_STREQ("foo", stat.values[2].key.c_str()) << "Bad label, adding a string with addf";

  EXPECT_STREQ("bool", stat.values[3].key.c_str()) << "Bad label, adding a true bool key with add";
  EXPECT_STREQ("True", stat.values[3].value.c_str()) << "Bad label, adding a true bool with add";

  EXPECT_STREQ("bool2", stat.values[4].key.c_str()) << "Bad label, adding a false bool key with add";
  EXPECT_STREQ("False", stat.values[4].value.c_str()) << "Bad label, adding a false bool with add";
}

TEST(DiagnosticUpdater, testDiagnosticStatusWrapperMergeSummary)
{
  DiagnosticStatusWrapper stat;

  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Old");
  stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "New");
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, stat.level) << "Bad level, merging levels (OK,OK)";
  EXPECT_STREQ("Old", stat.message.c_str()) << "Bad summary, merging levels (OK,OK)";

  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Old");
  stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "New");
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, stat.level) << "Bad level, merging levels (OK,WARN)";
  EXPECT_STREQ("New", stat.message.c_str()) << "Bad summary, merging levels (OK,WARN)";

  stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Old");
  stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "New");
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, stat.level) << "Bad level, merging levels (WARN,WARN)";
  EXPECT_STREQ("Old; New", stat.message.c_str()) << "Bad summary, merging levels (WARN,WARN)";

  stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Old");
  stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "New");
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, stat.level) << "Bad level, merging levels (WARN,ERROR)";
  EXPECT_STREQ("Old; New", stat.message.c_str()) << "Bad summary, merging levels (WARN,ERROR)";
}

TEST(DiagnosticUpdater, testFrequencyStatus)
{
  double minFreq = 10;
  double maxFreq = 20;
  
  ros::Time::init();
  ros::Time time(0, 0);
  ros::Time::setNow(time);

  FrequencyStatus fs(FrequencyStatusParam(&minFreq, &maxFreq, 0.5, 2));

  const int MS_TO_NS = 1000000;

  DiagnosticStatusWrapper stat[5];
  fs.tick();
  time += ros::Duration(0, 20 * MS_TO_NS); ros::Time::setNow(time);
  fs.run(stat[0]); // Should be too fast, 20 ms for 1 tick, lower limit should be 33ms.
  time += ros::Duration(0, 50 * MS_TO_NS); ros::Time::setNow(time);
  fs.tick();
  fs.run(stat[1]); // Should be good, 70 ms for 2 ticks, lower limit should be 66 ms.
  time += ros::Duration(0, 300 * MS_TO_NS); ros::Time::setNow(time);
  fs.tick();
  fs.run(stat[2]); // Should be good, 350 ms for 2 ticks, upper limit should be 400 ms.
  time += ros::Duration(0, 150 * MS_TO_NS); ros::Time::setNow(time);
  fs.tick();
  fs.run(stat[3]); // Should be too slow, 450 ms for 2 ticks, upper limit should be 400 ms.
  fs.clear();
  fs.run(stat[4]); // Should be good, just cleared it.

  using diagnostic_msgs::DiagnosticStatus;

  EXPECT_EQ(DiagnosticStatus::WARN, stat[0].level) << "max frequency exceeded but not reported";
  EXPECT_EQ(DiagnosticStatus::OK, stat[1].level) << "within max frequency but reported error";
  EXPECT_EQ(DiagnosticStatus::OK, stat[2].level) << "within min frequency but reported error";
  EXPECT_EQ(DiagnosticStatus::WARN, stat[3].level) << "min frequency exceeded but not reported";
  EXPECT_EQ(DiagnosticStatus::ERROR, stat[4].level) << "freshly cleared should fail";
  EXPECT_STREQ("", stat[0].name.c_str()) << "Name should not be set by FrequencyStatus";
  EXPECT_STREQ("Frequency Status", fs.getName().c_str()) << "Name should be \"Frequency Status\"";
}

TEST(DiagnosticUpdater, testSlowFrequencyStatus)
{
  // We have a slow topic (~0.5 Hz) and call the run() method once a second. This ensures that if the window size
  // is large enough (longer than 1/min_frequency * duration_between_run_calls), the diagnostics correctly reports
  // the frequency status even in time windows where no ticks happened.

  double minFreq = 0.25;
  double maxFreq = 0.75;

  ros::Time::init();
  ros::Time time(0, 0);
  ros::Time::setNow(time);

  FrequencyStatus fs(FrequencyStatusParam(&minFreq, &maxFreq, 0.0, 5));

  DiagnosticStatusWrapper stat[8];
  fs.tick();
  time += ros::Duration(1, 0); ros::Time::setNow(time);
  fs.run(stat[0]); // too high, 1 event in 1 sec window
  time += ros::Duration(1, 0); ros::Time::setNow(time);
  fs.run(stat[1]); // ok, 1 event in 2 sec window
  fs.tick();
  time += ros::Duration(1, 0); ros::Time::setNow(time);
  fs.run(stat[2]); // ok, 2 events in 3 sec window
  time += ros::Duration(1, 0); ros::Time::setNow(time);
  fs.run(stat[3]); // ok, 2 events in 4 sec window
  time += ros::Duration(1, 0); ros::Time::setNow(time);
  fs.run(stat[4]); // ok, 2 events in 5 sec window
  time += ros::Duration(1, 0); ros::Time::setNow(time);
  fs.run(stat[5]); // too low, 1 event in 5 sec window (first tick went out of window)
  time += ros::Duration(1, 0); ros::Time::setNow(time);
  fs.run(stat[6]); // too low, 1 event in 5 sec window (first tick went out of window)
  time += ros::Duration(1, 0); ros::Time::setNow(time);
  fs.run(stat[7]); // no events (second tick went out of window)
  time += ros::Duration(1, 0); ros::Time::setNow(time);

  using diagnostic_msgs::DiagnosticStatus;

  EXPECT_EQ(DiagnosticStatus::WARN, stat[0].level) << "max frequency exceeded but not reported";
  EXPECT_EQ(DiagnosticStatus::OK, stat[1].level) << "within frequency limits but reported error";
  EXPECT_EQ(DiagnosticStatus::OK, stat[2].level) << "within frequency limits but reported error";
  EXPECT_EQ(DiagnosticStatus::OK, stat[3].level) << "within frequency limits but reported error";
  EXPECT_EQ(DiagnosticStatus::OK, stat[4].level) << "within frequency limits but reported error";
  EXPECT_EQ(DiagnosticStatus::WARN, stat[5].level) << "min frequency exceeded but not reported";
  EXPECT_EQ(DiagnosticStatus::WARN, stat[6].level) << "min frequency exceeded but not reported";
  EXPECT_EQ(DiagnosticStatus::ERROR, stat[7].level) << "no events should fail";
}

TEST(DiagnosticUpdater, testTimeStampStatus)
{
  ros::Time::init();
  ros::Time time(1, 0);
  ros::Time::setNow(time);

  TimeStampStatus ts(DefaultTimeStampStatusParam);

  DiagnosticStatusWrapper stat[5];
  ts.run(stat[0]);
  ts.tick(time.toSec() + 2);
  ts.run(stat[1]);
  ts.tick(time);
  ts.run(stat[2]);
  ts.tick(time.toSec() - 4);
  ts.run(stat[3]);
  ts.tick(time.toSec() - 6);
  ts.run(stat[4]);

  using diagnostic_msgs::DiagnosticStatus;

  EXPECT_EQ(DiagnosticStatus::WARN, stat[0].level) << "no data should return a warning";
  EXPECT_EQ(DiagnosticStatus::ERROR, stat[1].level) << "too far future not reported";
  EXPECT_EQ(DiagnosticStatus::OK, stat[2].level) << "now not accepted";
  EXPECT_EQ(DiagnosticStatus::OK, stat[3].level) << "4 seconds ago not accepted";
  EXPECT_EQ(DiagnosticStatus::ERROR, stat[4].level) << "too far past not reported";
  EXPECT_STREQ("", stat[0].name.c_str()) << "Name should not be set by TimeStapmStatus";
  EXPECT_STREQ("Timestamp Status", ts.getName().c_str()) << "Name should be \"Timestamp Status\"";
}

TEST(DiagnosticUpdater, testSlowTimeStampStatus)
{
  // We have a slow topic (< 1 Hz) and call the run() method once a second. If we set the no_data_is_problem parameter
  // to false, updates without data should not generate a warning but should be treated as ok.

  ros::Time::init();
  ros::Time time(1, 0);
  ros::Time::setNow(time);

  SlowTimeStampStatus ts(TimeStampStatusParam(-1, 5));

  DiagnosticStatusWrapper stat[11];
  ts.run(stat[0]); // no events
  ts.tick(time.toSec() + 2);
  ts.run(stat[1]);
  ts.run(stat[2]);
  ts.tick(time.toSec() - 4);
  ts.run(stat[3]);
  ts.run(stat[4]);
  ts.run(stat[5]);
  ts.run(stat[6]);
  ts.tick(time.toSec() - 6);
  ts.run(stat[7]);
  ts.run(stat[8]);
  ts.run(stat[9]);
  ts.run(stat[10]);

  using diagnostic_msgs::DiagnosticStatus;

  EXPECT_EQ(DiagnosticStatus::OK, stat[0].level) << "no data should be ok";
  EXPECT_EQ(DiagnosticStatus::ERROR, stat[1].level) << "too far future not reported";
  EXPECT_EQ(DiagnosticStatus::OK, stat[2].level) << "no data should be ok";
  EXPECT_EQ(DiagnosticStatus::OK, stat[3].level) << "4 seconds ago not accepted";
  EXPECT_EQ(DiagnosticStatus::OK, stat[4].level) << "no data should be ok";
  EXPECT_EQ(DiagnosticStatus::OK, stat[5].level) << "no data should be ok";
  EXPECT_EQ(DiagnosticStatus::OK, stat[6].level) << "no data should be ok";
  EXPECT_EQ(DiagnosticStatus::ERROR, stat[7].level) << "too far past not reported";
  EXPECT_EQ(DiagnosticStatus::OK, stat[8].level) << "no data should be ok";
  EXPECT_EQ(DiagnosticStatus::OK, stat[9].level) << "no data should be ok";
  EXPECT_EQ(DiagnosticStatus::OK, stat[10].level) << "no data should be ok";
}

int main(int argc, char **argv){
  ros::init(argc, argv, "test_node");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
