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

#include <string>
#include <vector>
#include <memory>
#include "diagnostic_aggregator/ignore_analyzer.hpp"

//  using namespace diagnostic_aggregator;
//  using namespace std;

PLUGINLIB_EXPORT_CLASS(diagnostic_aggregator::IgnoreAnalyzer,
  diagnostic_aggregator::Analyzer)

diagnostic_aggregator::IgnoreAnalyzer::IgnoreAnalyzer() {}

diagnostic_aggregator::IgnoreAnalyzer::~IgnoreAnalyzer() {}

bool diagnostic_aggregator::IgnoreAnalyzer::init(
  const std::string base_name, const char * nsp,
  const rclcpp::Node::SharedPtr & n, const char * rnsp)
{
  return true;
}

std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>>
diagnostic_aggregator::IgnoreAnalyzer::report()
{
  std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> processed;
  return processed;
}
