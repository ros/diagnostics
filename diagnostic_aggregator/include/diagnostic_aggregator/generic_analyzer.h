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

/**!< \author Kevin Watts */

#ifndef GENERIC_ANALYZER_H
#define GENERIC_ANALYZER_H

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
#include "XmlRpcValue.h"

namespace diagnostic_aggregator {

/*!
 *\brief GenericAnalyzer is most basic diagnostic Analyzer
 * 
 * GenericAnalyzer analyzes diagnostics from list of topics and returns
 * processed diagnostics data. All analyzed status messages are prepended with
 * '/BasePath/MyPath', where FirstPrefix is common to all analyzers
 * (ex: 'PRE') and SecondPrefix is from this analyzer (ex: 'Power System').
 */
class GenericAnalyzer : public Analyzer
{
public:
  /*!
   *\brief Default constructor loaded by pluginlib
   */
  GenericAnalyzer();

  
  virtual ~GenericAnalyzer();

  /*!
   *\brief Initializes GenericAnalyzer from namespace
   *
   * NodeHandle is given private namespace to initialize (ex: ~Sensors)
   * Parameters of NodeHandle must follow this form. See DiagnosticAggregator
   * for instructions on passing these parameters to the aggregator.
   *\verbatim
   * PowerSystem:
   *   type: GenericAnalyzer
   *   prefix: Power System
   *   expected: [ 
   *     'IBPS 0',
   *     'IBPS 1']
   *   startswith: [
   *     'Smart Battery']
   *   name: [
   *     'Power Node 1018']
   *   contains: [
   *     'Battery']
   *\endverbatim
   *   
   *\param base_path : Prefix for all analyzers (ex: 'Robot')
   *\param n : NodeHandle in full namespace
   */
  bool init(const std::string base_path, const ros::NodeHandle &n);

  /*!
   *\brief Analyzes DiagnosticStatus messages
   * 
   *\return Vector of DiagnosticStatus messages. They must have the correct prefix for all names.
   */
  virtual std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > report();

  virtual bool analyze(const boost::shared_ptr<StatusItem> item);

  virtual bool match(const std::string name) const;

  /*!
   *\brief Returns full prefix (ex: "/Robot/Power System")
   */
  std::string getPath() const { return base_path_; }

  /*!
   *\brief Returns nice name (ex: "Power System")
   */
  std::string getName() const { return nice_name_; }

protected:
  std::string nice_name_;
  std::string base_path_;

private:
  double timeout_;
  int num_items_expected_;

  std::vector<std::string> chaff_; /**< Removed from the start of node names */
  std::vector<std::string> expected_;
  std::vector<std::string> startswith_;
  std::vector<std::string> contains_;
  std::vector<std::string> name_;
  std::vector<boost::regex> regex_;

  /*!
   *\brief Stores items by name
   */
  std::map<std::string, boost::shared_ptr<StatusItem> > items_;
};

}
#endif //GENERIC_ANALYZER_H
