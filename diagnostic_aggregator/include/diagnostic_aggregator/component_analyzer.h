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

///\author Kevin Watts

#ifndef COMPONENT_ANALYZER_H
#define COMPONENT_ANALYZER_H

#include <map>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/KeyValue.h"
#include "diagnostic_aggregator/analyzer.h"
#include "diagnostic_aggregator/status_item.h"
#include "XmlRpcValue.h"
#include "diagnostic_aggregator/component.h"

namespace diagnostic_aggregator {

/*!
 *\brief ComponentAnalyzer analyzers sensors devices on a PR2
 * 
 */
class ComponentAnalyzer : public Analyzer
{

public:
  /*!
   *\brief Default constructor loaded by pluginlib
   */
  ComponentAnalyzer();
  
  ~ComponentAnalyzer();

  /*!
   *\brief Initializes ComponentAnalyzer from namespace
   *
   *   
   *\param first_prefix : Prefix for all analyzers (ex: 'Robot')
   *\param n : NodeHandle in full namespace
   */
  bool init(std::string first_prefix, const ros::NodeHandle &n);


  /*!
   *\brief Analyzes DiagnosticStatus messages
   * 
   */
  std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > analyze(std::map<std::string, boost::shared_ptr<StatusItem> > msgs);

  /*!
   *\brief Returns full prefix (ex: "/Robot/Power System")
   */
  std::string getPrefix() { return full_prefix_; } 

  /*!
   *\brief Returns nice name (ex: "Power System")
   */
  std::string getName()  { return nice_name_; }

private:
  double timeout_;

  std::string nice_name_;
  std::string full_prefix_;

  std::vector<boost::shared_ptr<Component> > components_;

};


}
#endif // COMPONENT_ANALYZER_H
