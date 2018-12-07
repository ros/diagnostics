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

#ifndef DIAGNOSTIC_AGGREGATOR__ANALYZER_HPP_
#define DIAGNOSTIC_AGGREGATOR__ANALYZER_HPP_

#include <map>
#include <vector>
#include <string>
#include <memory>
#include "diagnostic_aggregator/status_item.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

// TODO(tfoote replace these terrible macros)
#define ROS_ERROR printf
#define ROS_FATAL printf
#define ROS_WARN printf
#define ROS_INFO printf

namespace diagnostic_aggregator
{

/*!
 *\brief Base class of all Analyzers. Loaded by aggregator.
 *
 * Base class, for all Analyzers loaded by pluginlib. All analyzers must
implement
 * these functions: init, match, analyze, report, getPath and getName.
 *
 * Analyzers must output their data in a vector of DiagnosticStatus messages
 * when the report() function is called. Each DiagnosticStatus message name
should
 * be prepended by the path of the Analyzer. The path is "BASE_PATH/MY_PATH" for
 * each analyzer. For example:
 * \verbatim
EtherCAT Device (head_pan_motor)
EtherCAT Device (head_tilt_motor)
---
PATH/EtherCAT Device (head_pan_motor)
PATH/EtherCAT Device (head_tilt_motor)
PATH/EtherCAT Devices
 * \endverbatim
 * Analyzers generally also output another DiagnosticStatus message for the
"header", with the
 * name BASE_PATH/MY_PATH, as in the example above ("PATH/EtherCAT Devices").
 *
 * For each new DiagnosticStatus name recieved, the analyzer will be asked
whether it wants
 * view the message using "match(string name)". If the analyzer wants to view
the message,
 * all future messages with that name will be given to the analyzer, using
"analyze(item)".
 *
 * If an analyzer is given a message to analyze, it should return true only if
it will report
 * that status value. For example, an Analyzer may look at all the messages for
robots motors
 * and power, but will only report messages on motors. This allows Analyzers to
use data from
 * other sections of the robot in reporting data.
 *
 * Since the match() function is called only when new DiagnosticStatus names
arrive, the
 * analyzers are not allowed to change which messages they want to look at.
 *
 */
class Analyzer
{
public:
  /*!
   *\brief Default constructor, called by pluginlib.
   */
  Analyzer() {}

  virtual ~Analyzer() {}

  /*!
   *\brief Analyzer is initialized with base path and namespace.
   *
   * The Analyzer initialized with parameters in its given
   * namespace. The "base_path" is common to all analyzers, and needs to be
   * prepended to all DiagnosticStatus names.
   *\param base_path : Common to all analyzers, prepended to all processed
   *names. Starts with "/". \param n : NodeHandle with proper private namespace
   *for analyzer.
   */
  // virtual bool init(const std::string base_path, const ros::NodeHandle &n) =
  // 0;
  //  virtual bool init(const std::string base_path, const
  //  rclcpp::Node::SharedPtr &n) = 0;
  virtual bool init(
    const std::string base_path, const char *,
    const rclcpp::Node::SharedPtr & n, const char *) = 0;

  /*!
   *\brief Returns true if analyzer will handle this item
   *
   * Match is called once for each new status name, so this return value cannot
   *change with time.
   */
  virtual bool match(const std::string name) = 0;

  /*!
   *\brief Returns true if analyzer will analyze this name
   *
   * This is called with every new item that an analyzer matches.
   * Analyzers should only return "true" if they will report the value of this
   * item. If it is only looking at an item, it should return false.
   */
  virtual bool analyze(const std::shared_ptr<StatusItem> item) = 0;

  /*!
   *\brief Analysis function, output processed data.
   *
   * report is called at 1Hz intervals. Analyzers should return a vector
   * of processed DiagnosticStatus messages.
   *
   *\return The array of DiagnosticStatus messages must have proper names, with
   *prefixes prepended
   */
  virtual std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>>
  report() = 0;

  /*!
   *\brief Returns full prefix of analyzer. (ex: '/Robot/Sensors')
   */
  virtual std::string getPath() const = 0;

  /*!
   *\brief Returns nice name for display. (ex: 'Sensors')
   */
  virtual std::string getName() const = 0;
};

}  // namespace diagnostic_aggregator
#endif  // DIAGNOSTIC_AGGREGATOR__ANALYZER_HPP_
