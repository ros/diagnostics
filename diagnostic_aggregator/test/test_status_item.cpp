/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Mahmoud Almasri
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
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *********************************************************************/

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "diagnostic_aggregator/status_item.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"

using diagnostic_aggregator::StatusItem;
using diagnostic_aggregator::Level_OK;
using diagnostic_aggregator::Level_Stale;
using diagnostic_msgs::msg::KeyValue;

namespace
{
KeyValue kv(const std::string & k, const std::string & v)
{
  KeyValue out;
  out.key = k;
  out.value = v;
  return out;
}
}  // namespace

TEST(StatusItem, constructorWithValuesInitializesAllFields)
{
  std::vector<KeyValue> values{kv("a", "1"), kv("b", "2")};
  StatusItem item("sensor", values, "ok-msg", Level_OK);

  EXPECT_EQ(item.getName(), "sensor");
  EXPECT_EQ(item.getMessage(), "ok-msg");
  EXPECT_EQ(item.getLevel(), Level_OK);
  EXPECT_EQ(item.getValue("a"), "1");
  EXPECT_EQ(item.getValue("b"), "2");
}

TEST(StatusItem, addValueNewKeyAppendsEntry)
{
  StatusItem item("sensor");
  ASSERT_FALSE(item.hasKey("k"));

  item.addValue("k", "v");

  EXPECT_TRUE(item.hasKey("k"));
  EXPECT_EQ(item.getValue("k"), "v");
}

TEST(StatusItem, addValueExistingKeyUpdatesInPlace)
{
  std::vector<KeyValue> values{kv("k", "old")};
  StatusItem item("sensor", values);

  item.addValue("k", "new");

  EXPECT_EQ(item.getValue("k"), "new");

  // values_ is private; check via the serialized message that no duplicate was added.
  auto msg = item.toStatusMsg("path");
  ASSERT_EQ(msg->values.size(), 1u);
  EXPECT_EQ(msg->values[0].key, "k");
  EXPECT_EQ(msg->values[0].value, "new");
}

TEST(StatusItem, hasKey)
{
  std::vector<KeyValue> values{kv("a", "1"), kv("b", "2")};
  StatusItem item("sensor", values);

  EXPECT_FALSE(item.hasKey("nope"));
  EXPECT_TRUE(item.hasKey("a"));
  EXPECT_TRUE(item.hasKey("b"));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
