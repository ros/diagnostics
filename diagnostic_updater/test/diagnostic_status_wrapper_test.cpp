/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2023, Robert Bosch GmbH
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

#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

TEST(DiagnosticStatusWrapper, testDiagnosticStatusWrapperDefaultConstructor) {
  // A default constructed DiagnosticStatusWrapper should be empty.
  diagnostic_updater::DiagnosticStatusWrapper dsw;
  EXPECT_EQ(dsw.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(dsw.message, "");
  EXPECT_EQ(dsw.values.size(), 0u);
}

TEST(DiagnosticStatusWrapper, testDiagnosticStatusWrapperSummary) {
  // A DiagnosticStatusWrapper constructed with a level should have that level and
  // an message.
  diagnostic_updater::DiagnosticStatusWrapper dsw;
  dsw.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Test");
  EXPECT_EQ(dsw.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(dsw.message, "Test");
  EXPECT_EQ(dsw.values.size(), 0u);
}

TEST(DiagnosticStatusWrapper, testDiagnosticStatusWrapperSummaryf) {
  // A DiagnosticStatusWrapper constructed with a level should have that level and
  // an formated message.
  diagnostic_updater::DiagnosticStatusWrapper dsw;
  dsw.summaryf(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Test %d", 42);
  EXPECT_EQ(dsw.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(dsw.message, "Test 42");
  EXPECT_EQ(dsw.values.size(), 0u);
}

TEST(DiagnosticStatusWrapper, testDiagnosticStatusWrapperClearSummary) {
  // A DiagnosticStatusWrapper should be empty after calling clearSummary().
  diagnostic_updater::DiagnosticStatusWrapper dsw;
  dsw.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Test");
  dsw.clearSummary();
  EXPECT_EQ(dsw.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(dsw.message, "");
  EXPECT_EQ(dsw.values.size(), 0u);
}

TEST(DiagnosticStatusWrapper, testDiagnosticStatusWrapperMergeSummary) {
  // A DiagnosticStatusWrapper should have the highest level and the longest
  // message after calling mergeSummary().
  diagnostic_updater::DiagnosticStatusWrapper dsw;
  dsw.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Was ok");
  EXPECT_EQ(dsw.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(dsw.message, "Was ok");
  EXPECT_EQ(dsw.values.size(), 0u);

  dsw.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Still ok");
  EXPECT_EQ(dsw.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(dsw.message, "Was ok");
  EXPECT_EQ(dsw.values.size(), 0u);

  dsw.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Warning");
  EXPECT_EQ(dsw.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(dsw.message, "Warning");
  EXPECT_EQ(dsw.values.size(), 0u);

  dsw.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Error");
  EXPECT_EQ(dsw.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(dsw.message, "Warning; Error");
  EXPECT_EQ(dsw.values.size(), 0u);
}

TEST(DiagnosticStatusWrapper, testDiagnosticStatusWrapperMergeFromInstance) {
  // A DiagnosticStatusWrapper should have the highest level and the longest
  // message after calling mergeSummary().
  diagnostic_updater::DiagnosticStatusWrapper dsw;
  dsw.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Was ok");
  EXPECT_EQ(dsw.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(dsw.message, "Was ok");
  EXPECT_EQ(dsw.values.size(), 0u);

  diagnostic_updater::DiagnosticStatusWrapper dsw2;
  dsw2.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Error");
  dsw.mergeSummary(dsw2);
  EXPECT_EQ(dsw.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(dsw.message, "Error");
  EXPECT_EQ(dsw.values.size(), 0u);
}

TEST(DiagnosticStatusWrapper, testDiagnosticStatusWrapperMergeSummaryf) {
  // A DiagnosticStatusWrapper can also be merged with a formated message.
  diagnostic_updater::DiagnosticStatusWrapper dsw;
  dsw.summaryf(diagnostic_msgs::msg::DiagnosticStatus::OK, "Was %s", "ok");
  EXPECT_EQ(dsw.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(dsw.message, "Was ok");
  EXPECT_EQ(dsw.values.size(), 0u);

  dsw.mergeSummaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Error %d", 42);
  EXPECT_EQ(dsw.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(dsw.message, "Error 42");
  EXPECT_EQ(dsw.values.size(), 0u);
}

TEST(DiagnosticStatusWrapper, testDiagnosticStatusAddString) {
  // A DiagnosticStatusWrapper should accept a String key-value pair.
  diagnostic_updater::DiagnosticStatusWrapper dsw;
  dsw.add("key", "value");
  EXPECT_EQ(dsw.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(dsw.message, "");
  EXPECT_EQ(dsw.values.size(), 1u);
  EXPECT_EQ(dsw.values[0].key, "key");
  EXPECT_EQ(dsw.values[0].value, "value");

  dsw.clear();
  EXPECT_EQ(dsw.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(dsw.message, "");
  EXPECT_EQ(dsw.values.size(), 0u);
}

TEST(DiagnosticStatusWrapper, testDiagnosticStatusAddf) {
  // A DiagnosticStatusWrapper should accept a formated String key-value pair.
  diagnostic_updater::DiagnosticStatusWrapper dsw;
  dsw.addf("key", "value %d", 42);
  EXPECT_EQ(dsw.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(dsw.message, "");
  EXPECT_EQ(dsw.values.size(), 1u);
  EXPECT_EQ(dsw.values[0].key, "key");
  EXPECT_EQ(dsw.values[0].value, "value 42");

  dsw.clear();
  EXPECT_EQ(dsw.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(dsw.message, "");
  EXPECT_EQ(dsw.values.size(), 0u);
}

TEST(DiagnosticStatusWrapper, testDiagnosticStatusAddBool) {
  // A DiagnosticStatusWrapper should accept a Bool key-value pair.
  diagnostic_updater::DiagnosticStatusWrapper dsw;
  dsw.add<bool>("key", true);
  EXPECT_EQ(dsw.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(dsw.message, "");
  EXPECT_EQ(dsw.values.size(), 1u);
  EXPECT_EQ(dsw.values[0].key, "key");
  EXPECT_EQ(dsw.values[0].value, "True");

  dsw.add<bool>("keyF", false);
  EXPECT_EQ(dsw.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(dsw.message, "");
  EXPECT_EQ(dsw.values.size(), 2u);
  EXPECT_EQ(dsw.values[1].key, "keyF");
  EXPECT_EQ(dsw.values[1].value, "False");

  dsw.clear();
  EXPECT_EQ(dsw.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(dsw.message, "");
  EXPECT_EQ(dsw.values.size(), 0u);
}
