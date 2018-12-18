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

#include <exception>
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_aggregator/aggregator.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    diagnostic_aggregator::Aggregator agg;

    rclcpp::Rate pub_rate(agg.getPubRate());
    while (agg.ok()) {
      rclcpp::spin_some(agg.get_node());
      agg.publishData();
      pub_rate.sleep();
    }
  } catch (std::exception & e) {
  std::cout << "Vaibhav diagnostic_aggregator exception hit  " << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
