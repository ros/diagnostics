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

#ifndef DIAGNOSTIC_AGGREGATOR_H
#define DIAGNOSTIC_AGGREGATOR_H

#include <ros/ros.h>
#include <string>
#include <map>
#include <vector>
#include <set>
#include <boost/shared_ptr.hpp>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include "XmlRpcValue.h"
#include "diagnostic_aggregator/analyzer.h"
#include "diagnostic_aggregator/analyzer_group.h"
#include "diagnostic_aggregator/status_item.h"
#include "diagnostic_aggregator/other_analyzer.h"


namespace diagnostic_aggregator {

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
 * Any other parametes in the namespace can by used to specify the analyzer. If
 * any analyzer is not properly specified, or returns false on initialization,
 * the aggregator will report the error and publish it in the aggregated output.
 */
class Aggregator
{ 
public:
  /*!
   *\brief Constructor initializes with main prefix (ex: '/Robot')
   */
  Aggregator();

  ~Aggregator();

  /*!
   *\brief Processes, publishes data. Should be called at pub_rate.
   */
  void publishData();

  /*!
   *\brief True if the NodeHandle reports OK
   */
  bool ok() const { return n_.ok(); }

  /*!
   *\brief Publish rate defaults to 1Hz, but can be set with ~pub_rate param
   */
  double getPubRate() const { return pub_rate_; }

private:
  ros::NodeHandle n_;
  ros::Subscriber diag_sub_; /**< DiagnosticArray, /diagnostics */
  ros::Publisher agg_pub_;  /**< DiagnosticArray, /diagnostics_agg */
  ros::Publisher toplevel_state_pub_;  /**< DiagnosticStatus, /diagnostics_toplevel_state */
  double pub_rate_;

  /*!
   *\brief Callback for incoming "/diagnostics"
   */
  void diagCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& diag_msg);

  AnalyzerGroup* analyzer_group_;

  OtherAnalyzer* other_analyzer_;

  std::string base_path_; /**< \brief Prepended to all status names of aggregator. */

  std::set<std::string> ros_warnings_;  /**< \brief Records all ROS warnings. No warnings are repeated. */

  /*
   *!\brief Checks timestamp of message, and warns if timestamp is 0 (not set)
   */
  void checkTimestamp(const diagnostic_msgs::DiagnosticArray::ConstPtr& diag_msg);

};

}

#endif // DIAGNOSTIC_AGGREGATOR_H
