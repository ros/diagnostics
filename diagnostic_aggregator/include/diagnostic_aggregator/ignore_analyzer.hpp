// Copyright 2015 Open Source Robotics Foundation, Inc.
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
#ifndef DIAGNOSTIC_AGGREGATOR__IGNORE_ANALYZER_HPP_
#define DIAGNOSTIC_AGGREGATOR__IGNORE_ANALYZER_HPP_

#include <memory>
#include <string>
#include <vector>
#include "diagnostic_aggregator/generic_analyzer.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"

// TODO(tfoote replace these terrible macros)
#define ROS_ERROR printf
#define ROS_FATAL printf
#define ROS_WARN printf
#define ROS_INFO printf

namespace diagnostic_aggregator
{

/*!
 *\brief IgnoreAnalyzer ignores all analyzer parameters and does nothing
 *
 * IgnoreAnalyzer is used to get rid of an Analyzer that is no longer part of a
 *robot configuration.
 *
 *\verbatim
 *<launch>
 *  <include file="$(find my_pkg)/my_analyzers.launch" />
 *
 *  <!-- Overwrite a specific Analyzer to discard all -->
 *  <param name="diag_agg/analyzers/motors/type" value="IgnoreAnalyzer" />
 *</launch>
 *\endverbatim
 *
 *
 */
class IgnoreAnalyzer : public Analyzer
{
public:
  /*!
   *\brief Default constructor loaded by pluginlib
   */
  IgnoreAnalyzer();

  virtual ~IgnoreAnalyzer();

  // bool init(const std::string base_name, const rclcpp::Node::SharedPtr &n );
  bool init(
    const std::string base_name, const char * nsp,
    const rclcpp::Node::SharedPtr & n, const char *);

  bool match(const std::string name) {return false;}

  bool analyze(std::shared_ptr<StatusItem> item) {return false;}

  /*
   *\brief Always reports an empty vector
   */
  virtual std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>>
  report();

  std::string getPath() const {return "";}
  std::string getName() const {return "";}
};

}  // namespace diagnostic_aggregator
#endif  // DIAGNOSTIC_AGGREGATOR__IGNORE_ANALYZER_HPP_
