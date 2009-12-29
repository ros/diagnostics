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

/**
 * \author Kevin Watts 
 */

#ifndef DIAGNOSTIC_ANALYZER_GROUP_H
#define DIAGNOSTIC_ANALYZER_GROUP_H

#include <map>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include "diagnostic_aggregator/status_item.h"
#include <boost/shared_ptr.hpp>
#include "XmlRpcValue.h"
#include "diagnostic_aggregator/analyzer.h"
#include "diagnostic_aggregator/status_item.h"
#include "pluginlib/class_loader.h"
#include "pluginlib/class_list_macros.h"

namespace diagnostic_aggregator {

class AnalyzerGroup : public Analyzer
{
public:
  AnalyzerGroup();
  
  virtual ~AnalyzerGroup();

  virtual bool init(const std::string base_path, const ros::NodeHandle &n);

  virtual bool match(const std::string name);

  virtual bool analyze(const boost::shared_ptr<StatusItem> item);

  virtual std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > report();

  std::string getPath() const { return path_; }
  
  std::string getName() const { return nice_name_; }

private:
  std::string path_, nice_name_;

  /*!
   *\brief Loads Analyzer plugins in "analyzers" namespace
   */
  pluginlib::ClassLoader<Analyzer> analyzer_loader_;

  std::vector<boost::shared_ptr<StatusItem> > aux_items_;

  std::vector<Analyzer*> analyzers_;

  std::map<const std::string, std::vector<bool> > matched_;

};

}

#endif //DIAGNOSTIC_ANALYZER_GROUP_H
