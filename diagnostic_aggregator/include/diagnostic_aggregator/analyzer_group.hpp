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

#ifndef DIAGNOSTIC_AGGREGATOR__ANALYZER_GROUP_HPP_
#define DIAGNOSTIC_AGGREGATOR__ANALYZER_GROUP_HPP_

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_aggregator/analyzer.hpp"
#include "diagnostic_aggregator/status_item.hpp"
#include "diagnostic_aggregator/visibility_control.hpp"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"

#include "rclcpp/rclcpp.hpp"

namespace diagnostic_aggregator
{
/*!
 *\brief Allows analyzers to be grouped together, or used as sub-analyzers
 *
 * The AnalyzerGroup is used by the diagnostic aggregator internally to
 * load and handle analyzers. It can be used as a normal analyzer plugin to
 * allow analyzers to become "sub-analyzers", or move as a group.
 *
 * The "sub-analyzers" are initialized using parameters in the "~analyzers"
 * namespace of the AnalyzerGroup. The "type" parameters determines the analyzer type.
 *
 * Example initialization:
 *\verbatim
 *  sensors:
 *  type: AnalyzerGroup
 *  path: Sensors
 *  analyzers:
 *    base_hk:
 *      type: GenericAnalyzer
 *      path: Base Hokuyo
 *      timeout: 5.0
 *      find_and_remove_prefix: base_hokuyo_node
 *      num_items: 3
 *    tilt_hk:
 *      type: GenericAnalyzer
 *      path: Tilt Hokuyo
 *      timeout: 5.0
 *      find_and_remove_prefix: tilt_hokuyo_node
 *      num_items: 3
 *    imu:
 *      type: GenericAnalyzer
 *      path: IMU
 *      timeout: 5.0
 *      find_and_remove_prefix: imu_node
 *      num_items: 3
 *\endverbatim
 *
 * Each namespace below "analyzers" describes a new Analyzer that will be loaded as a
 * sub-analyzer. Any analyzer that fails to initialize or loads incorrectly will
 * generate an error in the console output, and a special diagnostic item in the output
 * of the AnalyzerGroup that describes the error.
 *
 * In the above example, the AnalyzerGroup will have three sub-analyzers. The
 * AnalyzerGroup will report a DiagnosticStatus message in the processed output with
 * the name "Sensors" (the top-level state). The "Sensors" message will have the
 * level of the highest of the sub-analyzers, or the highest of "Sensors/Base Hokuyo",
 * "Sensors/Tilt Hokuyo" and "Sensors/IMU". The state of any other items, like
 * "Sensors/IMU/Connection" won't matter to the AnalyzerGroup.
 *
 * The Aggregator uses the AnalyzerGroup internally to load and update analyzers.
 *
 */
class AnalyzerGroup : public Analyzer
{
public:
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  AnalyzerGroup();

  DIAGNOSTIC_AGGREGATOR_PUBLIC
  virtual ~AnalyzerGroup();

  /*!
   *\brief Initialized with base path and namespace.
   *
   * The parameters in its namespace determine the sub-analyzers.
   */
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  virtual bool init(
    const std::string & base_path, const std::string & breadcrumb,
    const rclcpp::Node::SharedPtr node);

  /**!
   *\brief Add an analyzer to this analyzerGroup
   */
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  virtual bool addAnalyzer(std::shared_ptr<Analyzer> & analyzer);

  /**!
   *\brief Remove an analyzer from this analyzerGroup
   */
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  virtual bool removeAnalyzer(std::shared_ptr<Analyzer> & analyzer);

  /*!
   *\brief Match returns true if any sub-analyzers match an item
   */
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  virtual bool match(const std::string & name);

  /*!
   *\brief Clear match arrays. Used when analyzers are added or removed
   */
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  void resetMatches();

  /*!
   *\brief Analyze returns true if any sub-analyzers will analyze an item
   */
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  virtual bool analyze(const std::shared_ptr<StatusItem> item);

  /*!
   *\brief The processed output is the combined output of the sub-analyzers,
   * and the top level status
   */
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  virtual std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> report();

  virtual std::string getPath() const {return path_;}

  virtual std::string getName() const {return nice_name_;}

private:
  std::string path_;
  std::string nice_name_;
  std::string breadcrumb_;

  /*!
   *\brief Loads Analyzer plugins in "analyzers" namespace
   */
  pluginlib::ClassLoader<Analyzer> analyzer_loader_;

  rclcpp::Logger logger_;

  /*!
   *\brief These items store errors, if any, for analyzers that failed to initialize or load
   */
  std::vector<std::shared_ptr<StatusItem>> aux_items_;

  std::vector<std::shared_ptr<Analyzer>> analyzers_;

  /*
   *\brief The map of names to matchings is stored internally.
   */
  std::map<const std::string, std::vector<bool>> matched_;
};

}  // namespace diagnostic_aggregator

#endif  // DIAGNOSTIC_AGGREGATOR__ANALYZER_GROUP_HPP_
