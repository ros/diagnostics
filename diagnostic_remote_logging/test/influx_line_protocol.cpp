/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Daan Wijffels
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
 *   * Neither the name of the copyright holder nor the names of its
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

/**
 * \author Daan Wijffels
 */

#include "diagnostic_remote_logging/influx_line_protocol.hpp"

#include <gtest/gtest.h>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include <rclcpp/rclcpp.hpp>

diagnostic_msgs::msg::KeyValue createKeyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

// Test toInfluxTimestamp
TEST(InfluxTimestampTests, CorrectConversion)
{
  rclcpp::Time time(1672531200, 123456789);
  std::string expected = "1672531200123456789";
  EXPECT_EQ(toInfluxTimestamp(time), expected);
}

// Test escapeSpace
TEST(EscapeSpaceTests, HandlesSpaces)
{
  EXPECT_EQ(escapeSpace("test string"), "test\\ string");
  EXPECT_EQ(escapeSpace("no_space"), "no_space");
  EXPECT_EQ(escapeSpace("multiple spaces here"), "multiple\\ spaces\\ here");
}

// Test is_number
TEST(IsNumberTests, ValidatesNumbers)
{
  EXPECT_TRUE(is_number("123"));
  EXPECT_TRUE(is_number("123.456"));
  EXPECT_TRUE(is_number("-123.456"));
  EXPECT_FALSE(is_number("123abc"));
  EXPECT_FALSE(is_number("abc123"));
  EXPECT_FALSE(is_number(""));
}

// Test formatValues
TEST(FormatValuesTests, FormatsKeyValuePairs)
{
  std::vector<diagnostic_msgs::msg::KeyValue> values;
  values.push_back(createKeyValue("key1", "value"));
  values.push_back(createKeyValue("key2", "42"));
  values.push_back(createKeyValue("key3", "-3.14"));
  values.push_back(createKeyValue("key with spaces", "value with spaces"));

  std::string expected =
    "key1=\"value\",key2=42,key3=-3.14,key\\ with\\ "
    "spaces=\"value with spaces\"";
  EXPECT_EQ(formatValues(values), expected);
}

// Test splitHardwareID
TEST(SplitHardwareIDTests, SplitsCorrectly)
{
  EXPECT_EQ(splitHardwareID("node_name"), std::make_pair(std::string("none"), std::string("node_"
                                                                                          "name")));
  EXPECT_EQ(splitHardwareID("/ns/node_name"), std::make_pair(std::string("ns"), std::string("node_"
                                                                                            "nam"
                                                                                            "e")));
  EXPECT_EQ(splitHardwareID("/ns/prefix/node_name"),
            std::make_pair(std::string("ns"), std::string("prefix/"
                                                          "node_name")));
}

// Test statusToInfluxLineProtocol
TEST(StatusToInfluxLineProtocolTests, FormatsCorrectly)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.hardware_id = "/ns/node_name";
  status.level = 2;
  status.message = "Test message";
  status.values.push_back(createKeyValue("key1", "value1"));
  status.values.push_back(createKeyValue("key2", "42"));

  std::string expected =
    "node_name,ns=ns level=2,message=\"Test message\",key1=\"value1\",key2=42"
    " 1672531200123456789\n";
  std::string output;
  statusToInfluxLineProtocol(output, status, "1672531200123456789");

  EXPECT_EQ(output, expected);
}

// Test diagnosticArrayToInfluxLineProtocol
TEST(DiagnosticArrayToInfluxLineProtocolTests, HandlesMultipleStatuses)
{
  auto diag_msg = std::make_shared<diagnostic_msgs::msg::DiagnosticArray>();
  diag_msg->header.stamp = rclcpp::Time(1672531200, 123456789);

  diagnostic_msgs::msg::DiagnosticStatus status1;
  status1.hardware_id = "/ns1/node1";
  status1.level = 1;
  status1.message = "First status";
  status1.values.push_back(createKeyValue("keyA", "valueA"));

  diagnostic_msgs::msg::DiagnosticStatus status2;
  status2.hardware_id = "node2";
  status2.level = 2;
  status2.message = "Second status";
  status2.values.push_back(createKeyValue("keyB", "42"));

  diag_msg->status = {status1, status2};

  std::string expected =
    "node1,ns=ns1 level=1,message=\"First "
    "status\",keyA=\"valueA\" 1672531200123456789\n"
    "node2,ns=none level=2,message=\"Second "
    "status\",keyB=42 1672531200123456789\n";

  EXPECT_EQ(diagnosticArrayToInfluxLineProtocol(diag_msg), expected);
}

// Test diagnosticStatusToInfluxLineProtocol
TEST(DiagnosticStatusToInfluxLineProtocol, HandlesSingleStatus)
{
  auto status = std::make_shared<diagnostic_msgs::msg::DiagnosticStatus>();
  status->level = 1;
  status->name = "toplevel_state";
  auto time = rclcpp::Time(1672531200, 123456789);

  std::string expected = "toplevel_state level=1 1672531200123456789\n";
  EXPECT_EQ(diagnosticStatusToInfluxLineProtocol(status, time), expected);
}
