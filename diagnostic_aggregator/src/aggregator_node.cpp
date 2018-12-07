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

#include <diagnostic_aggregator/aggregator.h>
#include <exception>
#include "rclcpp/rclcpp.hpp"

using namespace std;
int main(int argc, char **argv)
{
 // ros::init(argc, argv, "diagnostic_aggregator");
  cout<< "Vaibhav diagnostic_aggregator init done "<< endl;
  rclcpp::init(argc, argv);
  try
  {
  cout<< "Vaibhav diagnostic_aggregator exception hit 1 "<< endl;
  diagnostic_aggregator::Aggregator agg;

  //ros::Rate pub_rate(agg.getPubRate());
   rclcpp::Rate  pub_rate(agg.getPubRate());
  //ros::Rate pub_rate(agg.getPubRate());
  while (agg.ok())
  {
   // ros::spinOnce();
    rclcpp::spin_some(agg.get_node());
    agg.publishData();
    pub_rate.sleep();
  }
  }
  catch (exception& e)
  {
  cout<< "Vaibhav diagnostic_aggregator exception hit  "<< endl;
   // ROS_FATAL("Diagnostic aggregator node caught exception. Aborting. %s", e.what());
   // ROS_BREAK();
  }
  
  cout<< "Vaibhav diagnostic_aggregator is going to shutdown  "<< endl;
 rclcpp::shutdown();
  return 0;
}
  
