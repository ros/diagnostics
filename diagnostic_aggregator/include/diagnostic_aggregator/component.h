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

#ifndef DIAG_COMPONENT_H
#define DIAG_COMPONENT_H

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


namespace diagnostic_aggregator {

/*!
 *\brief Removes redundant prefixes from status name. 
 *
 * Useful for cleaning up status names. 
 * Ex: /Hokuyo/Tilt HK/tilt_node: Connection to /Hokuyo/Tilt HK/Connection
 */
inline std::string removeLeadingNameChaff(const std::string input_name, const std::string chaff)
{
  std::string output_name = input_name;
  
  if (chaff.size() == 0)
    return output_name;

  // Remove start name from all output names
  // Turns "/PREFIX/base_hokuyo_node: Connection Status" to "/PREFIX/Connection Status"
  std::size_t last_slash = output_name.rfind("/");
  std::string start_of_name = output_name.substr(0, last_slash) + std::string("/");
  
  if (output_name.find(chaff) == last_slash + 1)
    output_name.replace(last_slash + 1, chaff.size(), "");
  
  if (output_name.find(":", last_slash) == last_slash + 1)
    output_name= start_of_name + output_name.substr(last_slash + 2);
  
  while (output_name.find(" ", last_slash) == last_slash + 1)
    output_name = start_of_name + output_name.substr(last_slash + 2);

  return output_name;
}

/*!
 *\brief Container class of PR2 components
 */
class Component
{
public:
  Component(std::string prefix, std::string name, double timeout);

  Component(std::string prefix, std::string name, double timeout, std::string start_name, XmlRpc::XmlRpcValue params);
  virtual ~Component() {}
  
  std::string getHeaderMsg() { return header_msg_; }
  std::string getName() { return nice_name_; }
  DiagnosticLevel getHeaderLevel() { return header_level_; }

  virtual std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > analyze(std::map<std::string, boost::shared_ptr<StatusItem> > msgs);
  
protected:
  std::string prefix_;
  std::string nice_name_;
  std::string start_name_;
  
  double timeout_;

  DiagnosticLevel header_level_;
  std::string header_msg_;

  std::map<std::string, boost::shared_ptr<StatusItem> > items_;  
  
  void updateItems(std::map<std::string, boost::shared_ptr<StatusItem> > msgs);
};

}

#endif //DIAG_COMPONENT_H
