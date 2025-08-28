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
#include <tuple>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"

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

std::tuple<std::string, std::string, std::string> splitName(const std::string & input)
{
  size_t colon_pos = input.find(':');
  std::string ns_node = (colon_pos != std::string::npos) ? input.substr(0, colon_pos) : input;
  std::string name = (colon_pos != std::string::npos) ? input.substr(colon_pos + 1) : "";

  // Trim leading whitespace from diagnostic name
  if (!name.empty() && name[0] == ' ') {
    name = name.substr(1);
  }

  // Handle use_fqn is false, only node name
  if (ns_node.empty() || ns_node[0] != '/') {
    // No leading slash - treat entire thing as node name
    return {"", ns_node, name};
  }

  // Remove leading slash when we get a fully qualified name
  ns_node = ns_node.substr(1);
  size_t last_slash = ns_node.rfind('/');

  std::string ns = ns_node.substr(0, last_slash);
  std::string node = ns_node.substr(last_slash + 1);

  return {ns, node, name};
}

void statusToInfluxLineProtocol(
  std::string & output, const diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & timestamp_str)
{
  auto [ns, node, name] = splitName(status.name);
  output += escapeSpace(node);
  if (!ns.empty()) {
    output += ",ns=" + escapeSpace(ns);
  }
  if (!name.empty()) {
    output += ",name=" + escapeSpace(name);
  }
  if (!status.hardware_id.empty()) {
    output += ",hardware_id=" + escapeSpace(status.hardware_id);
  }
  output += " level=" + std::to_string(status.level);

  if (!status.message.empty()) {
    output += ",message=\"" + status.message + "\"";
  }

  auto formatted_key_values = formatValues(status.values);
  if (!formatted_key_values.empty()) {
    output += "," + formatted_key_values;
  }
  output += " " + timestamp_str + "\n";
}

void statusToInfluxLineProtocol(
  std::string & output, const diagnostic_msgs::msg::DiagnosticStatus & status,
  const rclcpp::Time & time)
{
  statusToInfluxLineProtocol(output, status, toInfluxTimestamp(time));
}

void diagnosticArrayToInfluxLineProtocol(
  std::string & output, const diagnostic_msgs::msg::DiagnosticArray::SharedPtr & diag_msg)
{
  std::string timestamp = toInfluxTimestamp(diag_msg->header.stamp);
  for (auto & status : diag_msg->status) {
    statusToInfluxLineProtocol(output, status, timestamp);
  }
}

#endif  // DIAGNOSTIC_REMOTE_LOGGING__INFLUX_LINE_PROTOCOL_HPP_
