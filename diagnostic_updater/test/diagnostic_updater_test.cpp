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
#include <unistd.h>

using namespace diagnostic_updater;

class TestClass 
{
public: 
  void unwrapped(robot_msgs::DiagnosticStatus &s)
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

  private:
    void operator() (DiagnosticStatusWrapper &s) 
    {
      s.summary(0, "Test is running");
      s.addv("Value", 5);
      s.adds("String", "Toto");
      s.adds("Floating", 5.55);
      s.adds("Integer", 5);
      s.addsf("Formatted %s %i", "Hello", 5);
    }
  };
  
  TestClass c;
  ros::NodeHandle nh;
  
  // Note: Some of this code does nothing in terms of testing, but ensures
  // that all the constructors compile.
  DiagnosticUpdater<TestClass> dummy1(&c);
  DiagnosticUpdater<TestClass> dummy2(&c, *nh.getNode());
  DiagnosticUpdater<TestClass> updater(&c, nh);
  
  updater.addUpdater(&TestClass::unwrapped);
  updater.Updater::add("wrapped", &c, &TestClass::wrapped);
  
  classFunction cf;
  updater.Updater::add(cf);
}

TEST(DiagnosticUpdater, testDiagnosticStatusWrapper)
{
  DiagnosticStatusWrapper stat;
  
  const char *message = "dummy";
  int level = 1;
  stat.summary(level, message);
  EXPECT_STREQ(message, stat.message.c_str()) << "DiagnosticStatusWrapper::summary failed to set message";
  EXPECT_EQ(level, stat.level) << "DiagnosticStatusWrapper::summary failed to set level";

  stat.addv("toto", 5);
  stat.adds("baba", 5);
  stat.addsf("foo", "%05i", 27);
  
  EXPECT_EQ(5.0, stat.values[0].value) << "Bad value, adding a value with addv";
  EXPECT_STREQ("5", stat.strings[0].value.c_str()) << "Bad value, adding a string with adds";
  EXPECT_STREQ("00027", stat.strings[1].value.c_str()) << "Bad value, adding a string with addsf";
  EXPECT_STREQ("toto", stat.values[0].label.c_str()) << "Bad label, adding a value with addv";
  EXPECT_STREQ("baba", stat.strings[0].label.c_str()) << "Bad label, adding a string with adds";
  EXPECT_STREQ("foo", stat.strings[1].label.c_str()) << "Bad label, adding a string with addsf";
}

TEST(DiagnosticUpdater, testFrequencyStatus)
{
  double minFreq = 100;
  double maxFreq = 200;
  
  FrequencyStatus fs(minFreq, maxFreq, 0.5, 2);
  
  DiagnosticStatusWrapper stat[5];
  fs.tick();
  usleep(2000);
  fs(stat[0]); // Should be too fast, 2 ms for 1 tick, lower limit should be 3.3ms.
  usleep(5000);
  fs.tick();
  fs(stat[1]); // Should be good, 7 ms for 2 ticks, lower limit should be 6.6 ms.
  usleep(30000);
  fs.tick();
  fs(stat[2]); // Should be good, 35 ms for 2 ticks, upper limit should be 40 ms.
  usleep(15000);
  fs.tick();
  fs(stat[3]); // Should be too slow, 45 ms for 2 ticks, upper limit should be 40 ms.
  fs.clear();
  fs(stat[4]); // Should be good, just cleared it.

  EXPECT_EQ(2, stat[0].level) << "max frequency exceeded but not reported";
  EXPECT_EQ(0, stat[1].level) << "within max frequency but reported error";
  EXPECT_EQ(0, stat[2].level) << "within min frequency but reported error";
  EXPECT_EQ(2, stat[3].level) << "min frequency exceeded but not reported";
  EXPECT_EQ(2, stat[4].level) << "freshly cleared should fail";
  EXPECT_STREQ("", stat[0].name.c_str()) << "Name should not be set by FrequencyStatus";
  EXPECT_STREQ("Frequency Status", fs.getName().c_str()) << "Name should be FrequencyStatus";
}

int main(int argc, char **argv){
  ros::init(argc, argv, "test_node");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

