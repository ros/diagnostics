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

TEST(SelfTest, runSelfTest)
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string node_to_test;
  double max_delay;
  nh_private.param("node_to_test", node_to_test, std::string());
  nh_private.param("max_delay", max_delay, 60.);
  ASSERT_FALSE(node_to_test.empty()) << "selftest_rostest needs the \"node_to_test\" parameter.";
    
  std::string service_name = node_to_test+"/self_test";
  ros::service::waitForService(service_name, max_delay);

  diagnostic_msgs::SelfTest srv;
  
  if (nh.serviceClient<diagnostic_msgs::SelfTest>(service_name).call(srv))
  {
    diagnostic_msgs::SelfTest::Response &res = srv.response;
    
    std::string passfail;

    if (res.passed)
      passfail = "PASSED";
    else
      passfail = "FAILED";
    
    EXPECT_TRUE(res.passed) << "Overall self-test FAILED.";

    printf("Self test %s for device with id: [%s]\n", passfail.c_str(), res.id.c_str());

    for (size_t i = 0; i < res.status.size(); i++)
    {
      printf("%2d) %s\n", ((int) i + 1), res.status[i].name.c_str());
      if (res.status[i].level == 0)
        printf("     [OK]: ");
      else if (res.status[i].level == 1)
        printf("     [WARNING]: ");
      else
        printf("     [ERROR]: ");

      printf("%s\n", res.status[i].message.c_str());
    
      EXPECT_EQ(0, res.status[i].level) << res.status[i].name << " did not PASS: " << res.status[i].message;

      for (size_t j = 0; j < res.status[i].values.size(); j++)
        printf("      [%s] %s\n", res.status[i].values[j].key.c_str(), res.status[i].values[j].value.c_str());

      printf("\n");
    }
  }
  else
  {
    FAIL() << "Unable to trigger self-test.";
    printf("Failed to call service.\n");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "selftest_nodetest");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

