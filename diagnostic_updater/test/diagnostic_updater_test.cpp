// Copyright 2018 Persistent System Limited.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/update_functions.hpp>
#include <diagnostic_updater/DiagnosticStatusWrapper.hpp>
#include <unistd.h>

using namespace diagnostic_updater;

class TestClass 
{
public: 
  void unwrapped(diagnostic_msgs::msg::DiagnosticStatus &s)
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

  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Old");
  stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::OK, "New");
  EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::OK, stat.level) << "Bad level, merging levels (OK,OK)";
  EXPECT_STREQ("Old", stat.message.c_str()) << "Bad summary, merging levels (OK,OK)";

  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Old");
  stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "New");
  EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::WARN, stat.level) << "Bad level, merging levels (OK,WARN)";
  EXPECT_STREQ("New", stat.message.c_str()) << "Bad summary, merging levels (OK,WARN)";

  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Old");
  stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "New");
  EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::WARN, stat.level) << "Bad level, merging levels (WARN,WARN)";
  EXPECT_STREQ("Old; New", stat.message.c_str()) << "Bad summary, merging levels (WARN,WARN)";

  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Old");
  stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "New");
  EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::ERROR, stat.level) << "Bad level, merging levels (WARN,ERROR)";
  EXPECT_STREQ("Old; New", stat.message.c_str()) << "Bad summary, merging levels (WARN,ERROR)";
}

TEST(DiagnosticUpdater, testFrequencyStatus)
{
  double minFreq = 10;
  double maxFreq = 20;
  
  FrequencyStatus fs(FrequencyStatusParam(&minFreq, &maxFreq, 0.5, 2));
  
  DiagnosticStatusWrapper stat[5];
  fs.tick();
  usleep(20000);
  fs.run(stat[0]); // Should be too fast, 20 ms for 1 tick, lower limit should be 33ms.
  usleep(50000);
  fs.tick();
  fs.run(stat[1]); // Should be good, 70 ms for 2 ticks, lower limit should be 66 ms.
  usleep(300000);
  fs.tick();
  fs.run(stat[2]); // Should be good, 350 ms for 2 ticks, upper limit should be 400 ms.
  usleep(150000);
  fs.tick();
  fs.run(stat[3]); // Should be too slow, 450 ms for 2 ticks, upper limit should be 400 ms.
  fs.clear();
  fs.run(stat[4]); // Should be good, just cleared it.

  EXPECT_EQ(1, stat[0].level) << "max frequency exceeded but not reported";
  EXPECT_EQ(0, stat[1].level) << "within max frequency but reported error";
  EXPECT_EQ(0, stat[2].level) << "within min frequency but reported error";
  EXPECT_EQ(1, stat[3].level) << "min frequency exceeded but not reported";
  EXPECT_EQ(2, stat[4].level) << "freshly cleared should fail";
  EXPECT_STREQ("", stat[0].name.c_str()) << "Name should not be set by FrequencyStatus";
  EXPECT_STREQ("Frequency Status", fs.getName().c_str()) << "Name should be \"Frequency Status\"";
}

TEST(DiagnosticUpdater, testTimeStampStatus)
{
  TimeStampStatus ts(DefaultTimeStampStatusParam);

  DiagnosticStatusWrapper stat[5];
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
  EXPECT_STREQ("", stat[0].name.c_str()) << "Name should not be set by TimeStapmStatus";
  EXPECT_STREQ("Timestamp Status", ts.getName().c_str()) << "Name should be \"Timestamp Status\"";
}

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("test_node");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
