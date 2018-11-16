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

/**< \author Kevin Watts */

/**< \author Loads analyzer params, verifies that they are valid */

//#include <diagnostic_aggregator/aggregator.h>
#include <diagnostic_aggregator/analyzer_group.h>
//#include <ros/ros.h>
#include <string>
#include <gtest/gtest.h>
#include <iostream>

using namespace std;
//using namespace diagnostic_aggregator;

// Uses AnalyzerGroup to load analyzers
void v_TEST( )
{
 // ros::NodeHandle nh = ros::NodeHandle("~");
  auto context = rclcpp::contexts::default_context::get_global_default_context();
  const std::vector<std::string> arguments = {};
  const std::vector<rclcpp::Parameter> initial_values = {
	  rclcpp::Parameter("analyzer_params","")
     };
   const bool use_global_arguments = true;
   const bool use_intra_process = true;

   // ros::NodeHandle nh = ros::NodeHandle("~");
   auto nh = std::make_shared<rclcpp::Node>("analyzer_loader", "/", context, arguments, initial_values, use_global_arguments, use_intra_process);

  diagnostic_aggregator::AnalyzerGroup analyzer_group;
  std::string path = "base_path";
  // = new diagnostic_aggregator::AnalyzerGroup();
  if(analyzer_group.init(path, nh->get_namespace(),nh)){

cout<< "test passed" <<endl;	   
   }else{

cout<< "test failed" <<endl;	   
   }	  
  assert(analyzer_group.init(path, nh->get_namespace(),nh));
}

int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   v_TEST( );
 // rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("analyzer_loader");	
 // testing::InitGoogleTest(&argc, argv);
 
 // ros::init(argc, argv, "analyzer_loader");
  
  return 0;

}
