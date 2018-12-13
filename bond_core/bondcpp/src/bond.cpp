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

// Author: Stuart Glaser

#include <bondcpp/bond.hpp>

#ifdef _WIN32
#include <Rpc.h>
#else
#include <uuid/uuid.h>
#endif
#include <chrono>
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>
using namespace std::chrono_literals;

namespace bond
{
static std::string makeUUID()
{
#ifdef _WIN32
  UUID uuid;
  UuidCreate(&uuid);
  unsigned char * str;
  UuidToStringA(&uuid, &str);
  std::string return_string(reinterpret_cast<char *>(str));
  RpcStringFreeA(&str);
  return return_string;
#else
  uuid_t uuid;
  uuid_generate_random(uuid);
  char uuid_str[40];
  uuid_unparse(uuid, uuid_str);
  return std::string(uuid_str);
#endif
}

Bond::Bond(
  const std::string & topic, const std::string & id, rclcpp::Node::SharedPtr nh,
  std::function<void(void)> on_broken,
  std::function<void(void)> on_formed)
: bondsm_(new BondSM(this)),
  sm_(*bondsm_),
  topic_(topic),
  id_(id),
  instance_id_(makeUUID()),
  on_broken_(on_broken),
  on_formed_(on_formed),
  sisterDiedFirst_(false),
  started_(false),
  connect_timer_reset_flag_(false),
  disconnect_timer_reset_flag_(false),
  heartbeat_timer_reset_flag_(false),
  deadpublishing_timer_reset_flag_(false)
{
  nh_ = nh;
  setConnectTimeout(bond::msg::Constants::DEFAULT_CONNECT_TIMEOUT);
  setDisconnectTimeout(bond::msg::Constants::DEFAULT_DISCONNECT_TIMEOUT);
  setHeartbeatTimeout(bond::msg::Constants::DEFAULT_HEARTBEAT_TIMEOUT);
  setHeartbeatPeriod(bond::msg::Constants::DEFAULT_HEARTBEAT_PERIOD);
  setDeadPublishPeriod(bond::msg::Constants::DEAD_PUBLISH_PERIOD);
}

Bond::~Bond()
{
  if (!started_) {
    return;
  }
  breakBond();
  if (!waitUntilBroken(rclcpp::Duration(1.0 * 1e9))) {
    RCLCPP_DEBUG(nh_->get_logger(), "Bond failed to break on destruction %s (%s)",
      id_.c_str(), instance_id_.c_str());
  }

  // Must destroy the subscription before locking mutex_: shutdown()
  // will block until the status callback completes, and the status
  // callback locks mutex_ (in flushPendingCallbacks).
  // Stops the timers before locking the mutex.  Makes sure none of
  // the callbacks are running when we aquire the mutex.
  publishingTimerCancel();
  // deadpublishingTimerCancel();
  connectTimerCancel();
  heartbeatTimerCancel();
  disconnectTimerCancel();
  std::unique_lock<std::mutex> lock(mutex_);
  rclcpp::shutdown();
}

void Bond::setConnectTimeout(double dur)
{
  if (started_) {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot set timeouts after calling start()");
    return;
  }
  connect_timeout_ = dur * 1e9;  // conversion from seconds to nanoseconds
}

void Bond::connectTimerReset()
{
  rclcpp::Duration dur1(connect_timeout_);
  const std::chrono::nanoseconds period1(dur1.nanoseconds());
  // Callback function of connect timer
  auto connectTimerResetCallback =
    [this]() -> void
    {
      if (connect_timer_reset_flag_) {
        onConnectTimeout();
        connect_timer_->cancel();
        connect_timer_reset_flag_ = false;
      }    // flag is needed to have valid callback
    };
  // Connect timer started on node
  connect_timer_ = nh_->create_wall_timer(period1, connectTimerResetCallback);
}

void Bond::connectTimerCancel()
{
  connect_timer_->cancel();
}

void Bond::setDisconnectTimeout(double dur)
{
  if (started_) {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot set timeouts after calling start()");
    return;
  }

  disconnect_timeout_ = dur * 1e9;  // conversion from seconds to nanoseconds
}

void Bond::disconnectTimerReset()
{
  rclcpp::Duration dur2(disconnect_timeout_);
  const std::chrono::nanoseconds period2(dur2.nanoseconds());
  // Callback function of disconnect timer
  auto disconnectTimerResetCallback =
    [this]() -> void
    {
      if (disconnect_timer_reset_flag_) {
        onDisconnectTimeout();
        disconnect_timer_->cancel();
        disconnect_timer_reset_flag_ = false;
      }    // flag is needed to have valid callback
    };
  //  Disconnect timer started on node
  disconnect_timer_ = nh_->create_wall_timer(period2, disconnectTimerResetCallback);
}

void Bond::disconnectTimerCancel()
{
  disconnect_timer_->cancel();
}

void Bond::setHeartbeatTimeout(double dur)
{
  if (started_) {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot set timeouts after calling start()");
    return;
  }

  heartbeat_timeout_ = dur * 1e9;  //    conversion from seconds to nanoseconds
}

void Bond::heartbeatTimerReset()
{
  rclcpp::Duration dur3(heartbeat_timeout_);
  const std::chrono::nanoseconds period3(dur3.nanoseconds());
  //  Callback function of heartbeat timer
  auto heartbeatTimerResetCallback =
    [this]() -> void
    {
      if (heartbeat_timer_reset_flag_) {
        onHeartbeatTimeout();
        heartbeat_timer_->cancel();
        heartbeat_timer_reset_flag_ = false;
      }      //  flag is needed to have valid callback
    };
  //    heartbeat timer started on node
  heartbeat_timer_ = nh_->create_wall_timer(period3, heartbeatTimerResetCallback);
}

void Bond::heartbeatTimerCancel()
{
  heartbeat_timer_->cancel();
}

void Bond::setHeartbeatPeriod(double dur)
{
  if (started_) {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot set timeouts after calling start()");
    return;
  }

  heartbeat_period_ = dur * 1e9;     //  conversion from seconds to nanoseconds
}

void Bond::publishingTimerReset()
{
  rclcpp::Duration dur4(heartbeat_period_);
  const std::chrono::nanoseconds period4(dur4.nanoseconds());
  //  Callback function of publishing timer
  auto publishingTimerResetCallback =
    [this]() -> void
    {
      doPublishing();
    };
  //  publishing timer started on node
  publishing_timer_ = nh_->create_wall_timer(period4, publishingTimerResetCallback);
}

void Bond::publishingTimerCancel()
{
  publishing_timer_->cancel();
}

void Bond::setDeadPublishPeriod(double dur)
{
  if (started_) {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot set timeouts after calling start()");
    return;
  }

  dead_publish_period_ = dur * 1e9;  //  conversion from seconds to nanoseconds
}

void Bond::deadpublishingTimerReset()
{
  rclcpp::Duration dur5(dead_publish_period_);
  const std::chrono::nanoseconds period5(dur5.nanoseconds());
  //  callback function of dead publishing timer which will publish data when bond is broken
  auto deadpublishingTimerResetCallback =
    [this]() -> void
    {
      if (deadpublishing_timer_reset_flag_) {
        doPublishing();
        deadpublishing_timer_reset_flag_ = false;
      }     //  flag is needed to have valid callback
    };
  //  dead publishing timer started on node
  deadpublishing_timer_ = nh_->create_wall_timer(period5, deadpublishingTimerResetCallback);
}

void Bond::deadpublishingTimerCancel()
{
  deadpublishing_timer_->cancel();
}

/* TODO Callback Queue is not availabe in ROS2
void Bond::setCallbackQueue(rclcpp::CallbackQueueInterface *queue)
{
  if (started_) {
    RCLCPP_ERROR(nh_->get_logger(),"Cannot set callback queue after calling start()");
    return;
  }

  nh_.setCallbackQueue(queue);
}*/

void Bond::start()
{
  std::unique_lock<std::mutex> lock(mutex_);
  connect_timer_reset_flag_ = true;
  connectTimerReset();
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  //  set the depth to the QoS profile
  custom_qos_profile.depth = 5;
  pub_ = nh_->create_publisher<bond::msg::Status>(topic_, custom_qos_profile);
  sub_ = nh_->create_subscription<bond::msg::Status>(topic_,
      std::bind(&Bond::bondStatusCB, this, std::placeholders::_1));
  publishingTimerReset();
  started_ = true;
  heartbeatTimerReset();
  disconnectTimerReset();
  //  deadpublishingTimerReset();
}

void Bond::setFormedCallback(std::function<void(void)> on_formed)
{
  std::unique_lock<std::mutex> lock(mutex_);
  on_formed_ = on_formed;
}

void Bond::setBrokenCallback(std::function<void(void)> on_broken)
{
  std::unique_lock<std::mutex> lock(mutex_);
  on_broken_ = on_broken;
}

bool Bond::waitUntilFormed(rclcpp::Duration timeout)
{
  double time_conv = timeout.nanoseconds() * 1e9;
  rclcpp::Duration timeout1(time_conv);
  //  std::unique_lock<std::mutex> lock(mutex_);
  rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  rclcpp::Time deadline(steady_clock.now() + timeout1);

  while (sm_.getState().getId() == SM::WaitingForSister.getId()) {
    if (!rclcpp::ok()) {
      break;
    }
    rclcpp::Duration wait_time = rclcpp::Duration(0.1 * 1e9);
    if (timeout1 >= rclcpp::Duration(0.0 * 1e9)) {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      wait_time = std::min(wait_time, deadline - steady_clock.now());
    }
    if (wait_time <= rclcpp::Duration(0.0 * 1e9)) {
      break;  //  The deadline has expired
    }
    rclcpp::spin_some(nh_);
  }

  return sm_.getState().getId() != SM::WaitingForSister.getId();
}

bool Bond::waitUntilBroken(rclcpp::Duration timeout)
{
  double time_conv = timeout.nanoseconds() * 1e9;
  rclcpp::Duration timeout1(time_conv);
  //  std::unique_lock<std::mutex> lock(mutex_);
  rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  rclcpp::Time deadline(steady_clock.now() + timeout1);
  while (sm_.getState().getId() != SM::Dead.getId()) {
    if (!rclcpp::ok()) {
      break;
    }
    rclcpp::Duration wait_time = rclcpp::Duration(0.1 * 1e9);
    if (timeout1 >= rclcpp::Duration(0.0 * 1e9)) {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      wait_time = std::min(wait_time, deadline - steady_clock.now());
    }
    if (wait_time <= rclcpp::Duration(0.0 * 1e9)) {
      break;  //  The deadline has expired
    }

    rclcpp::spin_some(nh_);
  }

  return sm_.getState().getId() == SM::Dead.getId();
}

bool Bond::isBroken()
{
  std::unique_lock<std::mutex> lock(mutex_);
  return sm_.getState().getId() == SM::Dead.getId();
}

void Bond::breakBond()
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (sm_.getState().getId() != SM::Dead.getId()) {
      sm_.Die();
      publishStatus(false);
    }
  }
  flushPendingCallbacks();
}

void Bond::onConnectTimeout()
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    sm_.ConnectTimeout();
  }
  flushPendingCallbacks();
}

void Bond::onHeartbeatTimeout()
{
  //  Checks that heartbeat timeouts haven't been disabled globally.
  bool disable_heartbeat_timeout;
  nh_->get_parameter_or("bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM",
    disable_heartbeat_timeout, false);
  if (disable_heartbeat_timeout) {
    RCLCPP_WARN(nh_->get_logger(), "Heartbeat timeout is disabled.");
    RCLCPP_WARN(nh_->get_logger(), "Not breaking bond (topic: %s, id: %s)",
      topic_.c_str(), id_.c_str());
    return;
  }

  {
    std::unique_lock<std::mutex> lock(mutex_);
    sm_.HeartbeatTimeout();
  }
  flushPendingCallbacks();
}

void Bond::onDisconnectTimeout()
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    sm_.DisconnectTimeout();
  }
  flushPendingCallbacks();
}

void Bond::bondStatusCB(const bond::msg::Status::ConstSharedPtr msg)
{
  //  Filters out messages from other bonds and messages from ourself
  if (msg->id == id_ && msg->instance_id != instance_id_) {
    {
      std::unique_lock<std::mutex> lock(mutex_);

      if (sister_instance_id_.empty()) {
        sister_instance_id_ = msg->instance_id;
      }
    }

    if (msg->active) {
      sm_.SisterAlive();
    } else {
      sm_.SisterDead();
      //  Immediate ack for sister's death notification
      if (sisterDiedFirst_) {
        publishStatus(false);
      }
    }
  }
  flushPendingCallbacks();
}

void Bond::doPublishing()
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (sm_.getState().getId() == SM::WaitingForSister.getId() ||
    sm_.getState().getId() == SM::Alive.getId())
  {
    publishStatus(true);
  } else if (sm_.getState().getId() == SM::AwaitSisterDeath.getId()) {
    publishStatus(false);
  } else {
    publishingTimerCancel();
    //  deadpublishingTimerCancel();
  }
}

void Bond::publishStatus(bool active)
{
  auto msg_ = std::make_shared<bond::msg::Status>();
  rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  rclcpp::Time now = steady_clock.now();
  msg_->header.stamp = now;
  msg_->id = id_;
  msg_->instance_id = instance_id_;
  msg_->active = active;
  msg_->heartbeat_timeout = heartbeat_timeout_;
  msg_->heartbeat_period = heartbeat_period_;
  pub_->publish(msg_);
}

void Bond::flushPendingCallbacks()
{
  std::vector<std::function<void(void)>> callbacks;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    callbacks = pending_callbacks_;
    pending_callbacks_.clear();
  }

  for (size_t i = 0; i < callbacks.size(); ++i) {
    callbacks[i]();
  }
}

}  //  namespace bond


void BondSM::Connected()
{
  b->connectTimerCancel();
  b->condition_.notify_all();
  if (b->on_formed_) {
    b->pending_callbacks_.push_back(b->on_formed_);
  }
}

void BondSM::SisterDied()
{
  b->sisterDiedFirst_ = true;
}

void BondSM::Death()
{
  b->condition_.notify_all();
  b->heartbeatTimerCancel();
  b->disconnectTimerCancel();
  if (b->on_broken_) {
    b->pending_callbacks_.push_back(b->on_broken_);
  }
}

void BondSM::Heartbeat()
{
  b->heartbeat_timer_reset_flag_ = true;
  b->heartbeatTimerReset();
}

void BondSM::StartDying()
{
  b->heartbeatTimerCancel();
  b->disconnect_timer_reset_flag_ = true;
  b->disconnectTimerReset();
  /* TODO b->deadpublishing_timer_reset_flag_ = true;
  b->deadpublishingTimerReset();*/
}
