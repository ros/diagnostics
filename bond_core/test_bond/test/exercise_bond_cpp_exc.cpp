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
#include <unistd.h>
#include <gtest/gtest.h>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "bondcpp/bond.hpp"
#include "test_bond_srv_gen/srv/test_bond.hpp"

const char TOPIC[] = "test_bond_topic_exc";
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

void exercise_bond_cpp_exc()
{
  std::string id;
  rclcpp::Node::SharedPtr nh;
  nh = rclcpp::Node::make_shared("tets_bond");
  id = genId();
  bond::Bond a(TOPIC, id, nh);
  a.start();

  if (a.waitUntilFormed(rclcpp::Duration(5)) == false) {
    std::cout << " Test of waitUntilFormed succsessful... " << std::endl;
  } else {
    std::cout << " Test of waitUntilFormed faild..." << std::endl;
  }
  if (a.waitUntilBroken(rclcpp::Duration(10)) == true) {
    std::cout << " Test of waitUntilBroken succsessful... " << std::endl;
  } else {
    std::cout << " Test of waitUntilBroken faild..." << std::endl;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  exercise_bond_cpp_exc();
  rclcpp::shutdown();
  return 0;
}
