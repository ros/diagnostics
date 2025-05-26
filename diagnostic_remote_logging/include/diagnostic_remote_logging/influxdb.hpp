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

#ifndef DIAGNOSTIC_REMOTE_LOGGING__INFLUXDB_HPP_
#define DIAGNOSTIC_REMOTE_LOGGING__INFLUXDB_HPP_

#include <curl/curl.h>
#include <string>

#include "diagnostic_remote_logging/influx_line_protocol.hpp"

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

class InfluxDB : public rclcpp::Node
{
public:
  explicit InfluxDB(const rclcpp::NodeOptions & opt);
  ~InfluxDB();

private:
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr top_level_sub_;

  std::string post_url_, influx_token_;
  CURL * curl_;

  void setupConnection(const std::string & telegraf_url);

  void diagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void topLevelCallback(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg);

  bool sendToInfluxDB(const std::string & data);
};

#endif  // DIAGNOSTIC_REMOTE_LOGGING__INFLUXDB_HPP_
