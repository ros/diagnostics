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

#include <vector>
#include <memory>
#include "diagnostic_aggregator/discard_analyzer.hpp"

//  using namespace diagnostic_aggregator;
//  using namespace std;

PLUGINLIB_EXPORT_CLASS(diagnostic_aggregator::DiscardAnalyzer,
  diagnostic_aggregator::Analyzer)

diagnostic_aggregator::DiscardAnalyzer::DiscardAnalyzer() {}

diagnostic_aggregator::DiscardAnalyzer::~DiscardAnalyzer() {}

std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>>
diagnostic_aggregator::DiscardAnalyzer::report()
{
  std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> processed;
  return processed;
}
