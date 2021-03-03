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

/*!
 *\author Kevin Watts
 */

#ifndef DIAGNOSTIC_AGGREGATOR__AGGREGATOR_HPP_
#define DIAGNOSTIC_AGGREGATOR__AGGREGATOR_HPP_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "diagnostic_aggregator/analyzer.hpp"
#include "diagnostic_aggregator/analyzer_group.hpp"
#include "diagnostic_aggregator/other_analyzer.hpp"
#include "diagnostic_aggregator/status_item.hpp"
#include "diagnostic_aggregator/visibility_control.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "diagnostic_msgs/srv/add_diagnostics.hpp"

#include "rclcpp/rclcpp.hpp"

namespace diagnostic_aggregator
{
/*!
 *\brief Aggregator processes /diagnostics, republishes on /diagnostics_agg
 *
 * Aggregator is a node that subscribes to /diagnostics, processes it
 * and republishes aggregated data on /diagnostics_agg. The aggregator
 * creates a series of analyzers according to the specifications of its
 * private parameters. The aggregated diagnostics data is organized
 * in a tree structure. For example:
\verbatim
Input (status names):
  tilt_hokuyo_node: Frequency
  tilt_hokuyo_node: Connection
Output:
  /Robot
  /Robot/Sensors
  /Robot/Sensors/Tilt Hokuyo/Frequency
  /Robot/Sensors/Tilt Hokuyo/Connection
\endverbatim
 * The analyzer should always output a DiagnosticStatus with the name of the
 * prefix. Any other data output is up to the analyzer developer.
 *
 * Analyzer's are loaded by specifying the private parameters of the
 * aggregator.
\verbatim
base_path: My Robot
pub_rate: 1.0
other_as_errors: false
analyzers:
  sensors:
    type: GenericAnalyzer
    path: Tilt Hokuyo
    find_and_remove_prefix: tilt_hokuyo_node
  motors:
    type: PR2MotorsAnalyzer
  joints:
    type: PR2JointsAnalyzer
\endverbatim
 * Each analyzer is created according to the "type" parameter in its namespace.
 * Any other parameters in the namespace can by used to specify the analyzer. If
 * any analyzer is not properly specified, or returns false on initialization,
 * the aggregator will report the error and publish it in the aggregated output.
 */
class Aggregator
{
public:
  /*!
   *\brief Constructor initializes with main prefix (ex: '/Robot')
   */
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  Aggregator();

  DIAGNOSTIC_AGGREGATOR_PUBLIC
  virtual ~Aggregator();

  /*!
   *\brief Processes, publishes data. Should be called at pub_rate.
   */
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  void publishData();

  DIAGNOSTIC_AGGREGATOR_PUBLIC
  rclcpp::Node::SharedPtr get_node() const;

private:
  rclcpp::Node::SharedPtr n_;

  rclcpp::Logger logger_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  /// AddDiagnostics, /diagnostics_agg/add_diagnostics
  rclcpp::Service<diagnostic_msgs::srv::AddDiagnostics>::SharedPtr add_srv_;
  /// DiagnosticArray, /diagnostics
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  /// DiagnosticArray, /diagnostics_agg
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr agg_pub_;
  /// DiagnosticStatus, /diagnostics_toplevel_state
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr toplevel_state_pub_;
  std::mutex mutex_;
  double pub_rate_;
  int history_depth_;
  rclcpp::Clock::SharedPtr clock_;

  /*!
   *\brief Callback for incoming "/diagnostics"
   */
  void diagCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr diag_msg);

  std::unique_ptr<AnalyzerGroup> analyzer_group_;
  std::unique_ptr<OtherAnalyzer> other_analyzer_;

  std::string base_path_; /**< \brief Prepended to all status names of aggregator. */

  /// Records all ROS warnings. No warnings are repeated.
  std::set<std::string> ros_warnings_;

  /*
   *!\brief Checks timestamp of message, and warns if timestamp is 0 (not set)
   */
  void checkTimestamp(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr diag_msg);
};

}  // namespace diagnostic_aggregator

#endif  // DIAGNOSTIC_AGGREGATOR__AGGREGATOR_HPP_
