/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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

#include "ros/ros.h"
#include "diagnostic_msgs/SelfTest.h"

#include <gtest/gtest.h>
#include <string>

bool doTest(ros::NodeHandle nh)
{
  diagnostic_msgs::SelfTest srv;
  
  if (nh.serviceClient<diagnostic_msgs::SelfTest>("self_test").call(srv))
  {
    diagnostic_msgs::SelfTest::Response &res = srv.response;
    
    std::string passfail;

    if (res.passed)
      passfail = "PASSED";
    else
      passfail = "FAILED";

    printf("Self test %s for device with id: [%s]\n", passfail.c_str(), res.id.c_str());


    for (size_t i = 0; i < res.status.size(); i++)
    {
      printf("%2zd) %s\n", i + 1, res.status[i].name.c_str());
      if (res.status[i].level == 0)
        printf("     [OK]: ");
      else if (res.status[i].level == 1)
        printf("     [WARNING]: ");
      else
        printf("     [ERROR]: ");

      printf("%s\n", res.status[i].message.c_str());

      for (size_t j = 0; j < res.status[i].values.size(); j++)
        printf("      [%s] %s\n", res.status[i].values[j].key.c_str(), res.status[i].values[j].value.c_str());

      printf("\n");
    }
    return res.passed;
  }
  else
  {
    printf("Failed to call service.\n");
    return false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "run_selftest", ros::init_options::AnonymousName);
  if (argc != 2)
  {
    printf("usage: run_selftest name\n");
    return 1;
  }

  ros::NodeHandle nh(argv[1]);
  return !doTest(nh);
}

