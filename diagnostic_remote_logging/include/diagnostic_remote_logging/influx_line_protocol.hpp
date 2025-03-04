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

#ifndef DIAGNOSTIC_REMOTE_LOGGING__INFLUX_LINE_PROTOCOL_HPP_
#define DIAGNOSTIC_REMOTE_LOGGING__INFLUX_LINE_PROTOCOL_HPP_

#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

std::string toInfluxTimestamp(const rclcpp::Time & time)
{
  uint64_t seconds = static_cast<uint64_t>(time.seconds());
  uint64_t nanoseconds = static_cast<uint64_t>(time.nanoseconds()) % 1000000000;

  // Convert to strings
  std::string secStr = std::to_string(seconds);
  std::string nanosecStr = std::to_string(nanoseconds);

  // Zero-pad nanoseconds to 9 digits
  nanosecStr = std::string(9 - nanosecStr.length(), '0') + nanosecStr;

  return secStr + nanosecStr;
}

std::string escapeSpace(const std::string & input)
{
  std::string result;
  for (char c : input) {
    if (c == ' ') {
      result += '\\';  // Add a backslash before the space
    }
    result += c;  // Add the original character
  }
  return result;
}

bool is_number(const std::string & s)
{
  std::istringstream iss(s);
  double d;
  return iss >> std::noskipws >> d && iss.eof();
}

std::string formatValues(const std::vector<diagnostic_msgs::msg::KeyValue> & values)
{
  std::string formatted;
  for (const auto & kv : values) {
    if (kv.value.find("\n") != std::string::npos) {
      // If the value contains a newline, skip it
      // InfluxDB uses this to separate measurements
      continue;
    }

    formatted += escapeSpace(kv.key) + "=";

    if (is_number(kv.value)) {
      formatted += kv.value;
    } else {
      formatted += "\"" + kv.value + "\"";
    }
    formatted += ",";
  }
  if (!formatted.empty()) {
    formatted.pop_back();  // Remove the last comma
  }
  return formatted;
}

std::pair<std::string, std::string> splitHardwareID(const std::string & input)
{
  size_t first_slash_pos = input.find('/');

  // If no slash is found, treat the entire input as the node_name
  if (first_slash_pos == std::string::npos) {
    return {"none", input};
  }

  size_t second_slash_pos = input.find('/', first_slash_pos + 1);

  // If the second slash is found, extract the "ns" and "node" parts
  if (second_slash_pos != std::string::npos) {
    std::string ns = input.substr(first_slash_pos + 1, second_slash_pos - first_slash_pos - 1);
    std::string node = input.substr(second_slash_pos + 1);
    return {ns, node};
  }

  // If no second slash is found, everything after the first slash is the node
  std::string node = input.substr(first_slash_pos + 1);
  return {"none", node};    // ns is empty, node is the remaining string
}

void statusToInfluxLineProtocol(
  std::string & output,
  const diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & timestamp_str)
{
  // hardware_id is empty for analyzer groups, so skip them
  if (status.hardware_id.empty()) {
    return;
  }

  auto [ns, identifier] = splitHardwareID(status.hardware_id);
  output += escapeSpace(identifier) + ",ns=" + escapeSpace(ns) +
    " level=" + std::to_string(status.level) + ",message=\"" + status.message + "\"";
  auto formatted_key_values = formatValues(status.values);
  if (!formatted_key_values.empty()) {
    output += "," + formatted_key_values;
  }
  output += " " + timestamp_str + "\n";
}

std::string diagnosticStatusToInfluxLineProtocol(
  const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr & msg, const rclcpp::Time & time)
{
  std::string output =
    msg->name + " level=" + std::to_string(msg->level) + " " + toInfluxTimestamp(time) + "\n";
  return output;
}

std::string diagnosticArrayToInfluxLineProtocol(
  const diagnostic_msgs::msg::DiagnosticArray::SharedPtr & diag_msg)
{
  std::string output;
  std::string timestamp = toInfluxTimestamp(diag_msg->header.stamp);

  for (auto & status : diag_msg->status) {
    statusToInfluxLineProtocol(output, status, timestamp);
  }

  return output;
}

#endif  // DIAGNOSTIC_REMOTE_LOGGING__INFLUX_LINE_PROTOCOL_HPP_
