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

#ifndef DIAGNOSTIC_ANALYZER_H
#define DIAGNOSTIC_ANALYZER_H

#include <map>
#include <vector>
#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include "diagnostic_aggregator/diagnostic_item.h"

namespace diagnostic_analyzer {

/*!
 *\brief Base class of all DiagnosticAnalyzers. Loaded by aggregator.
 *
 * Base class, loaded by pluginlib. All analyzers must implement these
 * functions: init, analyze, getPrefix and getName.
 * 
 * Analyzers must output their data in a tree structure. The tree branches
 * are marked by '/' in the DiagnosticStatus name of the output. 
 * 
 * Each analyzer should output a "base" DiagnosticStatus, with the name of
 * "first_prefix/second_prefix" (ex: "/Robot/Motors")
 *
 */
class DiagnosticAnalyzer
{
public:
  /*!
   *\brief Default constructor, called by pluginlib.
   */
  DiagnosticAnalyzer() {}

  virtual ~DiagnosticAnalyzer() {}

  /*!
   *\brief DiagnosticAnalyzer is initialized with first prefix and namespace.
   *
   * The DiagnosticAnalyzer initialized with parameters in its given 
   * namespace. The "first_prefix" is common to all analyzers, and needs to be
   * prepended to all DiagnosticStatus names.
   *\param first_prefix : Common to all analyzers, prepended to all processed names. Starts with "/".
   *\param n : NodeHandle with proper private namespace for analyzer.
   */
  virtual bool init(std::string first_prefix, const ros::NodeHandle &n) 
  {
    ROS_FATAL("DiagnosticAnalyzer did not implement the init function");
    ROS_BREAK();
    return false;
  }

  /*!
   *\brief Analysis function, output processed data.
   *
   *\param msgs : The input map of messages, by status name. StatusPair stores message, count++ if analyzed. Returned array must be deleted by aggregator.
   */
  virtual std::vector<diagnostic_msgs::DiagnosticStatus*> analyze(std::map<std::string, diagnostic_item::DiagnosticItem*> msgs)
  {
    ROS_FATAL("DiagnosticAnalyzer did not implement the analyze function");
    ROS_BREAK();

    std::vector<diagnostic_msgs::DiagnosticStatus*> my_vec;
    return my_vec;
  }

  /*!
   *\brief Returns full prefix of analyzer. (ex: '/Robot/Sensors')
   */
  virtual std::string getPrefix() { return ""; }
  
  /*!
   *\brief Returns nice name for display. (ex: 'Sensors')
   */
  virtual std::string getName() { return ""; }

};

}

#endif //DIAGNOSTIC_ANALYZER_H
