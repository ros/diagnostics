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

#ifndef GENERIC_ANALYZER_H
#define GENERIC_ANALYZER_H

#include <map>
#include <ros/ros.h>
#include <vector>
#include <string>
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/KeyValue.h"
#include "diagnostic_aggregator/diagnostic_analyzer.h"
#include "diagnostic_aggregator/diagnostic_item.h"
#include "XmlRpcValue.h"

namespace diagnostic_analyzer {

/*!
 *\brief GenericAnalyzer is most basic DiagnosticAnalyzer
 * 
 * GenericAnalyzer analyzes diagnostics from list of topics and returns
 * processed diagnostics data. All analyzed status messages are prepended with
 * '/FirstPrefix/SecondPrefix', where FirstPrefix is common to all analyzers
 * (ex: 'PRE') and SecondPrefix is from this analyzer (ex: 'Power System').
 */
class GenericAnalyzer : public DiagnosticAnalyzer
{

public:
  /*!
   *\brief Default constructor loaded by pluginlib
   */
  GenericAnalyzer();

  
  ~GenericAnalyzer();

  /*!
   *\brief Initializes GenericAnalyzer from namespace
   *
   * NodeHandle is given private namespace to initialize (ex: ~Sensors)
   * Parameters of NodeHandle must follow this form. See DiagnosticAggregator
   * for instructions on passing these to the aggregator.
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
   *\param first_prefix : Prefix for all analyzers (ex: 'Robot')
   *\param n : NodeHandle in full namespace
   */
  bool init(std::string first_prefix, const ros::NodeHandle &n);

  /*!
   *\brief Initializes analyzer to deal with remaining data
   *
   * After all analyzers have been created this analyzer is created to 
   * process all remaining messages. It will prepend "first_prefix/Other"
   * to all messages that haven't been handled by other analyzers.
   * The "Other" analyzer is created automatically by the aggregator.
   */
  bool initOther(std::string first_prefix);

  /*!
   *\brief Analyzes DiagnosticStatus messages
   * 
   */
  std::vector<diagnostic_msgs::DiagnosticStatus*> analyze(std::map<std::string, diagnostic_item::DiagnosticItem*> msgs);

  /*!
   *\brief Returns full prefix (ex: "/Robot/Power System")
   */
  std::string getPrefix() { return full_prefix_; } 

  /*!
   *\brief Returns nice name (ex: "Power System")
   */
  std::string getName()  { return nice_name_; }
 
private:
  bool other_; /**< True if analyzer is supposed to analyze remaining messages */

  std::string nice_name_;
  std::string full_prefix_;

  std::vector<std::string> expected_;
  std::vector<std::string> startswith_;
  std::vector<std::string> contains_;
  std::vector<std::string> name_;

  /*!
   *\brief Stores items by name
   */
  std::map<std::string, diagnostic_item::DiagnosticItem*> items_;
  
  /*!
   *\brief Updates items_ with messages to analyze. Deletes to_analyze param.
   */
  void updateItems(std::vector<diagnostic_msgs::DiagnosticStatus*> to_analyze);
    
  /*!
   *\brief Returns items to be analyzed (items that haven't been already)
   */
  std::vector<diagnostic_msgs::DiagnosticStatus*> toAnalyzeOther(std::map<std::string, diagnostic_item::DiagnosticItem*> msgs);
    
  /*!
   *\brief Returns items that need to be analyzed
   */
  std::vector<diagnostic_msgs::DiagnosticStatus*> toAnalyze(std::map<std::string, diagnostic_item::DiagnosticItem*> msgs);


};


}
#endif //GENERIC_ANALYZER_H
