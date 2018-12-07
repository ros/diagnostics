// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef DIAGNOSTIC_AGGREGATOR_DISCARD_ANALYZER_H
#define DIAGNOSTIC_AGGREGATOR_DISCARD_ANALYZER_H

#include "diagnostic_aggregator/generic_analyzer.h"
#include <vector>
/*#include <boost/shared_ptr.hpp>*/
#include <memory>
#include "diagnostic_msgs/msg/diagnostic_status.hpp"


//TODO(tfoote replace these terrible macros)
#define ROS_ERROR printf
#define ROS_FATAL printf
#define ROS_WARN printf
#define ROS_INFO printf

namespace diagnostic_aggregator {

/*!
 *\brief DiscardAnalyzer is does not report any values. It is a subclass of GenericAnalyzer
 *
 * DiscardAnalyzer is a subclass of GenericAnalyzer. It will ignore any value that it matches.
 * It takes the any of the parameters of a GenericAnalyzer.
 *
 * It is useful for configuring an aggregator_node to ignore certain values in the diagnostics.
 *
 *\verbatim
 *<launch>
 *  <include file="$(find my_pkg)/my_analyzers.launch" />
 *
 *  <!-- Overwrite a specific Analyzer to discard all -->
 *  <param name="diag_agg/analyzers/motors/type" value="DiscardAnalyzer" />
 *</launch>
 *\endverbatim
 *
 *
 */
class DiscardAnalyzer : public GenericAnalyzer
{
public:
  /*!
   *\brief Default constructor loaded by pluginlib
   */
  DiscardAnalyzer();
  
  virtual ~DiscardAnalyzer();

  /*
   *\brief Always reports an empty vector
   */
  virtual std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus> > report();
};

}
#endif //GENERIC_ANALYZER_H
