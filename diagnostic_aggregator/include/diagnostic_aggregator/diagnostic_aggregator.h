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

// Author: Kevin Watts

#ifndef DIAGNOSTIC_AGGREGATOR_H
#define DIAGNOSTIC_AGGREGATOR_H

#include <ros/ros.h>
#include <string>
#include <map>
#include <vector>
#include <boost/scoped_ptr.hpp>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include "XmlRpcValue.h"
#include "pluginlib/class_loader.h"
#include "diagnostic_aggregator/diagnostic_analyzer.h"
#include "diagnostic_aggregator/diagnostic_item.h"
#include "diagnostic_aggregator/generic_analyzer.h"


namespace diagnostic_aggregator {

/*!
 *\brief DiagnosticAggregator processes /diagnostics, republishes on /diagnostics_agg
 *
 * DiagnosticAggregator is a node that subscribes to /diagnostics, processes it
 * and republishes aggregated data on /diagnostics_agg. The aggregator
 * creates a series of analyzers according to the specifications of its
 * private parameters. The aggregated diagnostics data is organized
 * in a tree structure. For example:
 *\verbatim
 *Input (status names):
 *  tilt_hokuyo_node Frequency
 *  tilt_hokuyo_node Connection
 *Output:
 *  /Robot
 *  /Robot/Sensors
 *  /Robot/Sensors/tilt_hokuyo_node Frequency
 *  /Robot/Sensors/tilt_hokuyo_node Connection
 *\endverbatim
 * The analyzer should always output a DiagnosticStatus with the name of the 
 * prefix. Any other data output is up to the analyzer developer.
 * 
 * DiagnosticAnalyzer's are loaded by specifying the private parameters of the
 * aggregator. 
 *\verbatim
 *sensors:
 *  type: GenericAnalyzer
 *  prefix: Sensors
 *  contains: [
 *    'hokuyo']
 *motors:
 *  type: PR2MotorsDiagnosticAnalyzer
 *joints:
 *  type: PR2JointsDiagnosticAnalyzer
 *other:
 *  type: GenericAnalyzer
 *  other: true
 *\endverbatim
 * Each analyzer is created according to the "type" parameter in its namespace.
 * Any other parametes in the namespace can by used to specify the analyzer. If
 * any analyzer is not properly specified, or returns false on initialization,
 * the aggregator program will exit.
 */
class DiagnosticAggregator
{
  
public:
  /*!
   *\brief Constructor initializes with main prefix (ex: '/Robot')
   */
   DiagnosticAggregator(std::string prefix);

  ~DiagnosticAggregator();

  /*!
   *\brief Loads private parameters, initializes analyzers.
   */
  void init();

  /*!
   *\brief Callback for "/diagnostics"
   */
  void diagCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& diag_msg);

  /*!
   *\brief Processes, publishes data. Should be called at 1Hz.
   */
  void publishData();

  ros::NodeHandle n_;


private:
  ros::Subscriber diag_sub_; /**< DiagnosticArray, /diagnostics */
  ros::Publisher agg_pub_;  /**< DiagnosticArray, /diagnostics_agg */

  /*!
   *\brief Loads DiagnosticAnalyzer plugins
   */
  pluginlib::ClassLoader<diagnostic_analyzer::DiagnosticAnalyzer> analyzer_loader_;

  
  std::vector<diagnostic_analyzer::DiagnosticAnalyzer*> analyzers_;

  std::map<std::string, diagnostic_item::DiagnosticItem*> msgs_;
  std::string prefix_; /**< Prepended to all status names of aggregator. */

  void clearMessages(); /**< Clears msgs_ map of (name, DiagnosticItems). */
};

}

#endif // DIAGNOSTIC_AGGREGATOR_H
