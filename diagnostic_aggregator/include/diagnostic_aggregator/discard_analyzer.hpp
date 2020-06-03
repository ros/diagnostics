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

#ifndef DIAGNOSTIC_AGGREGATOR__DISCARD_ANALYZER_HPP_
#define DIAGNOSTIC_AGGREGATOR__DISCARD_ANALYZER_HPP_

#include <memory>
#include <vector>

#include "diagnostic_aggregator/generic_analyzer.hpp"
#include "diagnostic_aggregator/visibility_control.hpp"

#include "diagnostic_msgs/msg/diagnostic_status.h"

namespace diagnostic_aggregator
{
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
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  DiscardAnalyzer();

  DIAGNOSTIC_AGGREGATOR_PUBLIC
  virtual ~DiscardAnalyzer();

  /*
   *\brief Always reports an empty vector
   */
  DIAGNOSTIC_AGGREGATOR_PUBLIC
  virtual std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> report();
};

}  // namespace diagnostic_aggregator

#endif  // DIAGNOSTIC_AGGREGATOR__DISCARD_ANALYZER_HPP_
