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
#include "diagnostic_aggregator/status_item.h"

namespace diagnostic_aggregator {

/*!
 *\brief Base class of all Analyzers. Loaded by aggregator.
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
class Analyzer
{
public:
  /*!
   *\brief Default constructor, called by pluginlib.
   */
  Analyzer() {}

  virtual ~Analyzer() {}

  /*!
   *\brief Analyzer is initialized with first prefix and namespace.
   *
   * The Analyzer initialized with parameters in its given 
   * namespace. The "base_path" is common to all analyzers, and needs to be
   * prepended to all DiagnosticStatus names.
   *\param base_path : Common to all analyzers, prepended to all processed names. Starts with "/".
   *\param n : NodeHandle with proper private namespace for analyzer.
   */
  virtual bool init(const std::string base_path, const ros::NodeHandle &n) = 0;

  /*!
   *\brief Returns true if analyzer will handle this item
   */
  virtual bool match(const std::string name) const = 0;

  /*!
   *\brief Returns true if analyzer will analyze this name
   *
   * This is called with every new item that an analyzer reports that it wants
   * to look at.
   */
  virtual bool analyze(const boost::shared_ptr<StatusItem> item) = 0;

  /*!
   *\brief Analysis function, output processed data.
   *
   * report is called at 1Hz intervals. Analyzers should return a vector 
   * of fully processed DiagnosticStatus messages.
   *
   *\return The array of DiagnosticStatus messages must have proper names, with prefixes prepended
   */
  virtual std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > report() = 0;


  /*!
   *\brief Returns full prefix of analyzer. (ex: '/Robot/Sensors')
   */
  virtual std::string getPath() const = 0;
  
  /*!
   *\brief Returns nice name for display. (ex: 'Sensors')
   */
  virtual std::string getName() const = 0;

};

}

#endif //DIAGNOSTIC_ANALYZER_H
