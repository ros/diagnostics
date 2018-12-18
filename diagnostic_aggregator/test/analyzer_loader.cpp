// Copyright 2018 Open Source Robotics Foundation, Inc.
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
/**< \author Kevin Watts */

/**< \author Loads analyzer params, verifies that they are valid */

#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include "diagnostic_aggregator/analyzer_group.hpp"

//  using namespace std;
//  using namespace diagnostic_aggregator;

//  Uses AnalyzerGroup to load analyzers
void v_TEST()
{
  //  ros::NodeHandle nh = ros::NodeHandle("~");
  auto context =
    rclcpp::contexts::default_context::get_global_default_context();
  const std::vector<std::string> arguments = {};
  const std::vector<rclcpp::Parameter> initial_values = {
    rclcpp::Parameter("analyzer_params", "")
  };
  const bool use_global_arguments = true;
  const bool use_intra_process = true;

  //  ros::NodeHandle nh = ros::NodeHandle("~");
  auto nh = std::make_shared<rclcpp::Node>(
    "analyzer_loader", "/", context, arguments, initial_values,
    use_global_arguments, use_intra_process);

  diagnostic_aggregator::AnalyzerGroup analyzer_group;
  std::string path = "base_path";
  //  = new diagnostic_aggregator::AnalyzerGroup();
  if (analyzer_group.init(path, nh->get_namespace(), nh, "gen_analyzers")) {
    std::cout << "test passed" << std::endl;
  } else {
    std::cout << "test failed" << std::endl;
  }
  assert(analyzer_group.init(path, nh->get_namespace(), nh, "gen_analyzers"));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  v_TEST();
  return 0;
}
