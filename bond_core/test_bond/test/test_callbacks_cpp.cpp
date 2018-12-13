// Copyright 2017 Open Source Robotics Foundation, Inc.
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

/*
 * Copyright (c) 2009, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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


#ifndef _WIN32
# include <uuid/uuid.h>
#else
# include <rpc.h>
#endif
#include <gtest/gtest.h>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "bondcpp/bond.hpp"
#include "test_bond_srv_gen/srv/test_bond.hpp"

const char TOPIC[] = "test_bond_topic";
std::string genId()
{
#ifndef _WIN32
  uuid_t uuid;
  uuid_generate_random(uuid);
  char uuid_str[40];
  uuid_unparse(uuid, uuid_str);
  return std::string(uuid_str);
#else
  UUID uuid;
  UuidCreate(&uuid);
  RPC_CSTR str;
  UuidToStringA(&uuid, &str);
  std::string return_string(reinterpret_cast<char *>(str));
  RpcStringFreeA(&str);
  return return_string;
#endif
}

class TestCallbacksCpp : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

/*TEST_F(TestCallbacksCpp, dieInLifeCallback)
{
  auto nh1 = rclcpp::Node::make_shared("test_callbacks_cpp");
  std::string id1 = genId();
  bond::Bond a(TOPIC, id1, nh1);
  bond::Bond b(TOPIC, id1, nh1);

  a.setFormedCallback(std::bind(&bond::Bond::breakBond, &a));
  a.start();
  b.start();

  EXPECT_TRUE(a.waitUntilFormed(rclcpp::Duration(5.0)));
  EXPECT_TRUE(b.waitUntilBroken(rclcpp::Duration(3.0)));
}*/

TEST_F(TestCallbacksCpp, remoteNeverConnects)
{
  auto nh2 = rclcpp::Node::make_shared("test_callbacks_cpp_2");
  std::string id2 = genId();
  bond::Bond a1(TOPIC, id2, nh2);

  a1.start();
  EXPECT_FALSE(a1.waitUntilFormed(rclcpp::Duration(5.0)));
  EXPECT_TRUE(a1.waitUntilBroken(rclcpp::Duration(10.0)));
}
