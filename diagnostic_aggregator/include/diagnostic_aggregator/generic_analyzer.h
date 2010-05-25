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
 * \author Kevin Watts 
 */

#ifndef DIAGNOSTIC_AGGREGATOR_GENERIC_ANALYZER_H
#define DIAGNOSTIC_AGGREGATOR_GENERIC_ANALYZER_H

#include <map>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include <boost/regex.hpp>
#include <pluginlib/class_list_macros.h>
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/KeyValue.h"
#include "diagnostic_aggregator/analyzer.h"
#include "diagnostic_aggregator/status_item.h"
#include "diagnostic_aggregator/generic_analyzer_base.h"
#include "XmlRpcValue.h"

namespace diagnostic_aggregator {

/*!
 *\brief Returns list of strings from a parameter
 *
 * Given an XmlRpcValue, gives vector of strings of that parameter
 *\return False if XmlRpcValue is not string or array of strings
 */
inline bool getParamVals(XmlRpc::XmlRpcValue param, std::vector<std::string> &output)
{
  XmlRpc::XmlRpcValue::Type type = param.getType();
  if (type == XmlRpc::XmlRpcValue::TypeString)
  {
    std::string find = param;
    output.push_back(find);
    return true;
  }
  else if (type == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (int i = 0; i < param.size(); ++i)
    {
      if (param[i].getType() != XmlRpc::XmlRpcValue::TypeString)
      {
        ROS_ERROR("Parameter is not a list of strings, found non-string value. XmlRpcValue: %s", param.toXml().c_str());
        output.clear();
        return false;
      }

      std::string find = param[i];
      output.push_back(find);
    }
    return true;
  }

  ROS_ERROR("Parameter not a list or string, unable to return values. XmlRpcValue:s %s", param.toXml().c_str());
  output.clear();
  return false;
}


/*!
 *\brief GenericAnalyzer is most basic diagnostic Analyzer
 * 
 * GenericAnalyzer analyzes a segment of diagnostics data and reports
 * processed diagnostics data. All analyzed status messages are prepended with
 * "Base Path/My Path", where "Base Path" is from the parent of this Analyzer,
 * (ex: 'PR2') and "My Path" is from this analyzer (ex: 'Power System').
 *
 * The GenericAnalyzer is initialized as a plugin by the diagnostic Aggregator.
 * Following is an example of the necessary parameters of the GenericAnalyzer.
 * See the Aggregator class for more information on loading Analyzer plugins.
 *\verbatim
 * my_namespace:
 *   type: GenericAnalyzer
 *   path: My Path
 *\endverbatim
 * Required Parameters:
 * - \b type This is the class name of the analyzer, used to load the correct plugin type.
 * - \b path All diagnostic items analyzed by the GenericAnalyzer will be under "Base Path/My Path". 
 * 
 * In the above example, the GenericAnalyzer wouldn't analyze anything. The GenericAnalyzer 
 * must be configured to listen to diagnostic status names. To do this, optional parameters,
 * like "contains", will tell the analyzer to analyze an item that contains that value. The 
 * GenericAnalyzer looks at the name of the income diagnostic_msgs/DiagnosticStatus messages
 * to determine item matches.
 * 
 * Optional Parameters for Matching:
 * - \b contains Any item that contains these values
 * - \b startswith Item name must start with this value
 * - \b name Exact name match
 * - \b expected Exact name match, will warn if not present
 * - \b regex Regular expression (regex) match against name
 * The above parameters can be given as a single string ("tilt_hokuyo_node") or a list
 * of strings (['Battery', 'Smart Battery']).
 *
 * In some cases, it's possible to clean up the processed diagnostic status names.
 * - \b remove_prefix If these prefix is found in a status name, it will be removed in the output. 
 * Can be given as a string or list of strings.
 *
 * The special parameter '''find_and_remove_prefix''' combines "startswith" and 
 * "remove_prefix". It can be given as a string or list of strings.
 * 
 * If the number of incoming items under a GenericAnalyzer is known, use '''num_items'''
 * to set an exact value. If the number of items that matches the above parameters is 
 * incorrect, the GenericAnalyzer will report an error in the top-level status. This is 
 * "-1" by default. Negative values will not cause a check on the number of items.
 *
 * For tracking stale items, use the "timeout" parameter. Any item that doesn't update
 * within the timeout will be marked as "Stale", and will cause an error in the top-level
 * status. Default is 5.0 seconds. Any value <0 will cause stale items to be ignored.
 *
 * The GenericAnalyzer can discard stale items. Use the "discard_stale" parameter to
 * remove any items that haven't updated within the timeout. This is "false" by default.
 *
 * Example configurations:
 *\verbatim
 * hokuyo:
 *   type: GenericAnalyzer
 *   path: Hokuyo
 *   find_and_remove_prefix: hokuyo_node
 *   num_items: 3
 *\endverbatim
 *
 *\verbatim
 * power_system:
 *   type: GenericAnalyzer
 *   path: Power System
 *   startswith: [
 *     'Battery',
 *     'IBPS']
 *   expected: Power board 1000
 *   dicard_stale: true
 *\endverbatim
 *
 * \subsubsection GenericAnalyzer Behavior
 * 
 * The GenericAnalyzer will report the latest status of any item that is should analyze.
 * It will report a separate diagnostic_msgs/DiagnosticStatus with the name 
 * "Base Path/My Path". This "top-level" status will have the error state of the highest 
 * of its children. 
 * 
 * Stale items are handled differently. A stale child will cause an error
 * in the top-level status, but if all children are stale, the top-level status will 
 * be stale.
 * 
 * Example analyzer behavior, using the "Hokuyo" configuration above:
 *\verbatim
 * Input - (DiagnosticStatus Name, Error State)
 * hokuyo_node: Connection Status, OK
 * hokuyo_node: Frequency Status, Warning
 * hokuyo_node: Driver Status, OK
 *
 * Output - (DiagnosticStatus Name, Error State)
 * Hokuyo, Warning
 * Hokuyo/Connection Status, OK
 * Hokuyo/Frequency Status, Warning
 * Hokuyo/Driver Status, OK
 *\endverbatim
 *
 *
 */
class GenericAnalyzer : public GenericAnalyzerBase
{
public:
  /*!
   *\brief Default constructor loaded by pluginlib
   */
  GenericAnalyzer();
  
  virtual ~GenericAnalyzer();

  // Move to class description above
  /*!
   *\brief Initializes GenericAnalyzer from namespace. Returns true if s
   *
   *\param base_path : Prefix for all analyzers (ex: 'Robot')
   *\param n : NodeHandle in full namespace
   *\return True if initialization succeed, false if no errors of 
   */
  bool init(const std::string base_path, const ros::NodeHandle &n);

  /*!
   *\brief Reports current state, returns vector of formatted status messages
   * 
   *\return Vector of DiagnosticStatus messages, with correct prefix for all names.
   */
  virtual std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > report();

  /*!
   *\brief Returns true if item matches any of the given criteria
   * 
   */
  virtual bool match(const std::string name);

private:
  std::vector<std::string> chaff_; /**< Removed from the start of node names. */
  std::vector<std::string> expected_;
  std::vector<std::string> startswith_;
  std::vector<std::string> contains_;
  std::vector<std::string> name_;
  std::vector<boost::regex> regex_; /**< Regular expressions to check against diagnostics names. */

};

}
#endif //DIAGNOSTIC_AGGREGATOR_GENERIC_ANALYZER_H
