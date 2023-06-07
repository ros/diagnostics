/*
 * Copyright (c) 2019, Willow Garage, Inc.
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

/*!
 *\author Kevin Watts
 *\brief Simple node with a self test that reports an error
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>

#include "selftest_fixture.hpp"
#include "selftest_node.hpp"

class ErrorSelftestNode : public SelftestNode
{
public:
  ErrorSelftestNode()
  : SelftestNode("error_selftest_node")
  {}

  void setup_test_cases() override
  {
    self_test_.add<SelftestNode>("Pretest", this, &SelftestNode::pretest);

    self_test_.add<SelftestNode>("ID Lookup", this, &SelftestNode::test1);
    self_test_.add("Error generating test", this, &ErrorSelftestNode::test2);
    self_test_.add<SelftestNode>("Value generating test", this, &SelftestNode::test3);
    self_test_.add<SelftestNode>("Value testing test", this, &SelftestNode::test4);

    self_test_.add<SelftestNode>("Posttest", this, &SelftestNode::pretest);
  }

  void test2(diagnostic_updater::DiagnosticStatusWrapper & status) override
  {
    // Report an error, this should fail self test
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Reporting an error");
  }
};

// using directive necessary as gtest macro TEST_F gets confused with template classes
using Fixture = SelftestFixture<ErrorSelftestNode>;
TEST_F(Fixture, run_self_test)
{
  auto client = node_->create_client<diagnostic_msgs::srv::SelfTest>(
    "error_selftest_node/self_test");

  using namespace std::chrono_literals;
  if (!client->wait_for_service(5s)) {
    FAIL() << "could not connect to self test service";
  }

  auto request = std::make_shared<diagnostic_msgs::srv::SelfTest::Request>();

  using ServiceResponseFuture =
    rclcpp::Client<diagnostic_msgs::srv::SelfTest>::SharedFuture;
  auto response_received_callback = [](ServiceResponseFuture future) {
      auto result_out = future.get();

      EXPECT_FALSE(result_out->passed) << "ErrorSelfTestNode shall not pass!";
      EXPECT_STREQ(std::to_string(12345).c_str(), result_out->id.c_str());
      unsigned char max_level = 0;
      for (const auto & status : result_out->status) {
        max_level = std::max(status.level, max_level);
        auto some_val = std::find_if(
          status.values.begin(), status.values.end(), [](auto it) {
            return it.key == "some val";
          });
        if (some_val != status.values.end()) {
          EXPECT_EQ(std::to_string(42), some_val->value);
        }
      }
      EXPECT_TRUE(max_level > 0) <<
        "Self test failed, but no sub tests reported a failure or warning";
    };
  auto future = client->async_send_request(request, response_received_callback);
  if (!future.valid()) {
    FAIL() << "could not correctly send self test service request";
  }
  rclcpp::spin_until_future_complete(node_, future);
}
