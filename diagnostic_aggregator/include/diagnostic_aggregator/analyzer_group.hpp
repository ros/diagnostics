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
#ifndef DIAGNOSTIC_AGGREGATOR__ANALYZER_GROUP_HPP_
#define DIAGNOSTIC_AGGREGATOR__ANALYZER_GROUP_HPP_

#include <algorithm>
#include <map>
#include <string>
#include <vector>
#include <memory>
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_aggregator/analyzer.hpp"
#include "diagnostic_aggregator/status_item.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"

// TODO(tfoote replace these terrible macros)
#define ROS_ERROR printf
#define ROS_FATAL printf
#define ROS_WARN printf
#define ROS_INFO printf
#define ROS_ASSERT_MSG printf
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
 * namespace of the AnalyzerGroup. The "type" parameters determines the analyzer
 *type.
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
 * Each namespace below "analyzers" describes a new Analyzer that will be loaded
 *as a sub-analyzer. Any analyzer that fails to initialize or loads incorrectly
 *will generate an error in the console output, and a special diagnostic item in
 *the output of the AnalyzerGroup that describes the error.
 *
 * In the above example, the AnalyzerGroup will have three sub-analyzers. The
 * AnalyzerGroup will report a DiagnosticStatus message in the processed output
 *with the name "Sensors" (the top-level state). The "Sensors" message will have
 *the level of the highest of the sub-analyzers, or the highest of "Sensors/Base
 *Hokuyo", "Sensors/Tilt Hokuyo" and "Sensors/IMU". The state of any other
 *items, like "Sensors/IMU/Connection" won't matter to the AnalyzerGroup.
 *
 * The Aggregator uses the AnalyzerGroup internally to load and update
 *analyzers.
 *
 */
class AnalyzerGroup : public Analyzer
{
public:
  AnalyzerGroup();

  virtual ~AnalyzerGroup();

  /*!
   *\brief Initialized with base path and namespace.
   *
   * The parameters in its namespace determine the sub-analyzers.
   */
  /*  virtual bool init(const std::string base_path, const ros::NodeHandle
   * &n);*/

  virtual bool init(
    const std::string base_path, const char *,
    const rclcpp::Node::SharedPtr & n, const char *);
  // virtual bool init(const std::string base_path, const
  // rclcpp::Node::SharedPtr &n); //change to remove dependecy of copy
  // constructor

  /**!
   *\brief Add an analyzer to this analyzerGroup
   */
  virtual bool addAnalyzer(std::shared_ptr<Analyzer> & analyzer);

  /**!
   *\brief Remove an analyzer from this analyzerGroup
   */
  virtual bool removeAnalyzer(std::shared_ptr<Analyzer> & analyzer);

  /*!
   *\brief Match returns true if any sub-analyzers match an item
   */
  virtual bool match(const std::string name);

  /*!
   *\brief Clear match arrays. Used when analyzers are added or removed
   */
  void resetMatches();

  /*!
   *\brief Analyze returns true if any sub-analyzers will analyze an item
   */
  virtual bool analyze(const std::shared_ptr<StatusItem> item);

  /*!
   *\brief The processed output is the combined output of the sub-analyzers, and
   *the top level status
   */
  virtual std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>>
  report();

  virtual std::string getPath() const {return path_;}

  virtual std::string getName() const {return nice_name_;}

private:
  std::string path_, nice_name_;

  /*!
   *\brief Loads Analyzer plugins in "analyzers" namespace
   */
  pluginlib::ClassLoader<Analyzer> analyzer_loader_;

  /*!
   *\brief These items store errors, if any, for analyzers that failed to
   *initialize or load
   */
  std::vector<std::shared_ptr<StatusItem>> aux_items_;

  std::vector<std::shared_ptr<Analyzer>> analyzers_;

  /*
   *\brief The map of names to matchings is stored internally.
   */
  std::map<const std::string, std::vector<bool>> matched_;
  rclcpp::Node::SharedPtr analyzers_nh;
  rclcpp::Node::SharedPtr analyzers_nh1;
};

}  // namespace diagnostic_aggregator

#endif  // DIAGNOSTIC_AGGREGATOR__ANALYZER_GROUP_HPP_
