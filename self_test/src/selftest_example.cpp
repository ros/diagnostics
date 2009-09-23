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


#include <ros/node.h>

#include "diagnostic_msgs/SelfTest.h"

#include "self_test/self_test.h"

#include <stdexcept>

//  using namespace std;

class MyNode : public ros::Node
{
public:

  // The SelfTest must be templated by your node
  SelfTest<MyNode> self_test_;

  // A value showing statefulness of tests
  double some_val;

  // During construction, the self_test_ takes a pointer to your node
  MyNode() : ros::Node("my_node"), self_test_(this)
  {
    // A pretest can be added which will run before all other tests.
    // NOTE: It is only run once for the entire test sequence
    self_test_.setPretest(  &MyNode::pretest );

    // A pretest can be added which will run after all other tests.
    // NOTE: It is only run once for the entire test sequence
    self_test_.setPosttest( &MyNode::pretest );

    // This differs philosophically from other testing architectures
    // such as gtest which would run your setup and teardown on each
    // test.  I have chosen this approach because we often use tests
    // in sequence bringing up the hardware device and want to see
    // the status of each of those stages sequentially.

    // Tests added will be run in the order in which they are added
    self_test_.addTest(     &MyNode::test1   );
    self_test_.addTest(     &MyNode::test2   );
    self_test_.addTest(     &MyNode::test3   );
    self_test_.addTest(     &MyNode::test4   );
  }

  void pretest()
  {
    printf("Doing preparation stuff before we run our test.\n");
    
    some_val = 1.0;
  }


  // All tests take a reference to a DiagnosticStatus message which they should populate
  // The default values are status.level = 2 (ERROR), and status.message = "No message was set"
  void test1(diagnostic_msgs::DiagnosticStatus& status)
  {
    // Good practice is to set the name of the test first
    status.name = "ID Lookup";

    // Look up ID here
    char ID[] = "12345";
    bool lookup_successful = true;

    if (lookup_successful)
    {
      status.level = 0;
      status.message = "ID Lookup successful";
      
      // Using setID on the selftest pushes the ID to an accessible location 
      self_test_.setID(ID);

    } else {
      status.level = 2;
      status.message = "ID Lookup failed";
    }
  }

  // Tests do not necessarily need to catch their exceptions.
  void test2(diagnostic_msgs::DiagnosticStatus& status)
  {
    status.name = "Exception generating test";

    // Note, we start setting our status to success.  Since our
    // exception is not caught, however, the SelfTest class will
    // change level to ERROR.  This wouldn't be common practice And I
    // would instead recommend not changing the status to success
    // until after the exception might be generated.

    status.level = 0;

    // Exceptions of time runtime_error will actually propagate messages
    throw std::runtime_error("we did something that threw an exception");

    // Here's where we would report success if we'd made it past
    status.level = 0;
    status.message = "We made it past the exception throwing statement.";
  }

  // The state of the node can be changed as the tests are operating
  void test3(diagnostic_msgs::DiagnosticStatus& status)
  {
    status.name = "Value generating test";

    // Do something that changes the state of the node
    some_val += 41.0;

    status.set_values_size(1);
    status.values[0].value = some_val;
    status.values[0].key = "some value";

    status.level = 0;
    status.message = "We successfully changed the value.";
  }

  void test4(diagnostic_msgs::DiagnosticStatus& status)
  {
    status.name = "Value testing test";

    if (some_val == 42.0)
    {
      status.level = 0;
      status.message = "We observed the change in value";
    } 
    else
    {
      status.level = 2;
      status.message = "We failed to observe the change in value";
    }
  }

  void posttest()
  {
    printf("Doing cleanup stuff after we run our test.\n");
  }

  bool spin()
  {
    while (ok())
    {
      //Do something important...
      usleep(1000000);
      
      // Calling checkTest tells the SelfTest class that it's ok
      // to actually run the test.  This gives you flexibility to
      // keep the SelfTest service from necessarily interrupting
      // other operations.
      self_test_.checkTest();
    }
    return true;
  }

};

int
main(int argc, char** argv)
{
  ros::init(argc, argv);

  MyNode n;

  n.spin();

  

  return(0);
}
