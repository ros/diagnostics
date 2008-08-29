/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <iostream>
#include <ros/node.h>

#include "robot_srvs/SelfTest.h"

#include "self_test/self_test.h"
#include <pthread.h>

#include <vector>

using namespace std;

class EmptyNode: public ros::node
{
public:

  EmptyNode() : ros::node("empty_node")
  {
    advertise_service("~self_test", &EmptyNode::doTest, this);
  }

  bool doTest(robot_srvs::SelfTest::request &req,
              robot_srvs::SelfTest::response &res)
  {
    std::vector<robot_msgs::DiagnosticStatus> status_vec;

    robot_msgs::DiagnosticStatus status;

    status.name = "None";
    status.level = 2;
    status.message = "No message was set";

    status.set_values_size(1);
    status.values[0].value_label = "value 1";
    status.values[0].value       = 1.0;


    status_vec.push_back(status);

    res.set_status_vec(status_vec);

    return true;
  }

};

int
main(int argc, char** argv)
{
  ros::init(argc, argv);

  EmptyNode en;

  en.spin();

  ros::fini();

  return(0);
}
