/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/**
 * \author Kevin Watts
 */

#ifndef DIAGNOSTIC_AGGREGATOR__ANALYZER_HPP_
#define DIAGNOSTIC_AGGREGATOR__ANALYZER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_aggregator/status_item.hpp"
#include "diagnostic_aggregator/visibility_control.hpp"

#include "diagnostic_msgs/msg/diagnostic_status.h"
#include "diagnostic_msgs/msg/key_value.h"

#include "rclcpp/rclcpp.hpp"

namespace diagnostic_aggregator
{
/*!
 *\brief Base class of all Analyzers. Loaded by aggregator.
 *
 * Base class, for all Analyzers loaded by pluginlib. All analyzers must implement
 * these functions: init, match, analyze, report, getPath and getName.
 *
 * Analyzers must output their data in a vector of DiagnosticStatus messages
 * when the report() function is called. Each DiagnosticStatus message name should
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
 * Analyzers generally also output another DiagnosticStatus message for the "header", with the
 * name BASE_PATH/MY_PATH, as in the example above ("PATH/EtherCAT Devices").
 *
 * For each new DiagnosticStatus name recieved, the analyzer will be asked whether it wants
 * view the message using "match(string name)". If the analyzer wants to view the message,
 * all future messages with that name will be given to the analyzer, using "analyze(item)".
 *
 * If an analyzer is given a message to analyze, it should return true only if it will report
 * that status value. For example, an Analyzer may look at all the messages for robots motors
 * and power, but will only report messages on motors. This allows Analyzers to use data from
 * other sections of the robot in reporting data.
 *
 * Since the match() function is called only when new DiagnosticStatus names arrive, the
 * analyzers are not allowed to change which messages they want to look at.
 *
 */
class Analyzer
{
public:
  /*!
   *\brief Default constructor, called by pluginlib.
   */
  Analyzer()
  : clock_(std::make_shared<rclcpp::Clock>()) {}

  virtual ~Analyzer() {}

  /*!
   *\brief Analyzer is initialized with base path and namespace.
   *
   * The Analyzer initialized with parameters in its given
   * namespace. The "base_path" is common to all analyzers, and needs to be
   * prepended to all DiagnosticStatus names.
   *\param base_path : Common to all analyzers, prepended to all processed names. Starts with "/".
   *\param breadcrumb : Prefix for parameter getter.
   *\param n : NodeHandle with proper private namespace for analyzer.
   */
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  virtual bool init(
    const std::string & base_path, const std::string & breadcrumb,
    const rclcpp::Node::SharedPtr node) = 0;

  /*!
   *\brief Returns true if analyzer will handle this item
   *
   * Match is called once for each new status name, so this return value cannot change
   * with time.
   */
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  virtual bool match(const std::string & name) = 0;

  /*!
   *\brief Returns true if analyzer will analyze this name
   *
   * This is called with every new item that an analyzer matches.
   * Analyzers should only return "true" if they will report the value of this
   * item. If it is only looking at an item, it should return false.
   */
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  virtual bool analyze(const std::shared_ptr<StatusItem> item) = 0;

  /*!
   *\brief Analysis function, output processed data.
   *
   * report is called at 1Hz intervals. Analyzers should return a vector
   * of processed DiagnosticStatus messages.
   *
   *\return The array of DiagnosticStatus messages must have proper names, with prefixes prepended
   */
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  virtual std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> report() = 0;

  /*!
   *\brief Returns full prefix of analyzer. (ex: '/Robot/Sensors')
   */
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  virtual std::string getPath() const = 0;

  /*!
   *\brief Returns nice name for display. (ex: 'Sensors')
   */
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  virtual std::string getName() const = 0;

protected:
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace diagnostic_aggregator

#endif  // DIAGNOSTIC_AGGREGATOR__ANALYZER_HPP_
