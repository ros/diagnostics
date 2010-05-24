/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#ifndef DIAGNOSTIC_AGGREGATOR_IGNORE_ANALYZER_H
#define DIAGNOSTIC_AGGREGATOR_IGNORE_ANALYZER_H

#include "diagnostic_aggregator/generic_analyzer.h"
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <ros/ros.h>

namespace diagnostic_aggregator {

/*!
 *\brief IgnoreAnalyzer ignores all analyzer parameters and does nothing
 *
 * IgnoreAnalyzer is used to get rid of an Analyzer that is no longer part of a robot configuration.
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

  bool init(const std::string base_name, const ros::NodeHandle &n);

  bool match(const std::string name) { return false; }

  bool analyze(boost::shared_ptr<StatusItem> item) { return false; }

  /*
   *\brief Always reports an empty vector
   */
  virtual std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > report();

  std::string getPath() const { return ""; }
  std::string getName() const { return ""; }
};

}
#endif // DIAGNOSTIC_AGGREGATOR_IGNORE_ANALYZER_H
