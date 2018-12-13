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

/** \author Stuart Glaser */

#ifndef BONDCPP__BOND_HPP_
#define BONDCPP__BOND_HPP_

#include <memory>
#include <mutex>
#include <functional>
#include <condition_variable>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "bond/msg/constants.hpp"
#include "bond/msg/status.hpp"
#include "BondSM_sm.hpp"

namespace bond
{
/** \brief Forms a bond to monitor another process.
 *
 * The bond::Bond class implements a bond, allowing you to monitor
 * another process and be notified when it dies.  In turn, it will be
 * notified when you die.
 */
class Bond
{
public:
  /** \brief Constructs a bond, but does not connect
   *
   * \param topic The topic used to exchange the bond status messages.
   * \param id The ID of the bond, which should match the ID used on
   *           the sister's end.
   * \param on_broken callback that will be called when the bond is broken.
   * \param on_formed callback that will be called when the bond is formed.
   */
  Bond(
    const std::string & topic, const std::string & id, rclcpp::Node::SharedPtr nh,
    std::function<void(void)> on_broken = std::function<void(void)>(),
    std::function<void(void)> on_formed = std::function<void(void)>());

  /** \brief Destructs the object, breaking the bond if it is still formed.
   */
  ~Bond();

  double getConnectTimeout() const {return connect_timeout_;}
  void setConnectTimeout(double dur);
  void connectTimerReset();
  void connectTimerCancel();
  double getDisconnectTimeout() const {return disconnect_timeout_;}
  void setDisconnectTimeout(double dur);
  void disconnectTimerReset();
  void disconnectTimerCancel();
  double getHeartbeatTimeout() const {return heartbeat_timeout_;}
  void setHeartbeatTimeout(double dur);
  void heartbeatTimerReset();
  void heartbeatTimerCancel();
  double getHeartbeatPeriod() const {return heartbeat_period_;}
  void setHeartbeatPeriod(double dur);
  void publishingTimerReset();
  void publishingTimerCancel();
  double getDeadPublishPeriod() const {return dead_publish_period_;}
  void setDeadPublishPeriod(double dur);
  void deadpublishingTimerReset();
  void deadpublishingTimerCancel();

#if 0
  void setCallbackQueue(ros::CallbackQueueInterface * queue);
#endif
  /** \brief Starts the bond and connects to the sister process.
   */
  void start();
  /** \brief Sets the formed callback.
   */
  void setFormedCallback(std::function<void(void)> on_formed);

  /** \brief Sets the broken callback
   */
  void setBrokenCallback(std::function<void(void)> on_broken);

  /** \brief Blocks until the bond is formed for at most 'duration'.
   *
   * \param timeout Maximum duration to wait.  If -1 then this call will not timeout.
   * \return true iff the bond has been formed.
   */
  bool waitUntilFormed(rclcpp::Duration timeout = rclcpp::Duration(-1 * 1e9));
  /** \brief Blocks until the bond is formed for at most 'duration'.
   *
   * \param timeout Maximum duration to wait.  If -1 then this call will not timeout.
   * \return true iff the bond has been formed.
   */
  bool waitUntilBroken(rclcpp::Duration timeout = rclcpp::Duration(-1 * 1e9));

  /** \brief Blocks until the bond is broken for at most 'duration'.
   *
   * \param timeout Maximum duration to wait.  If -1 then this call will not timeout.
   * \return true iff the bond has been broken, even if it has never been formed.
   */
  bool isBroken();

  /** \brief Breaks the bond, notifying the other process.
   */
  void breakBond();
  std::string getTopic() {return topic_;}
  std::string getId() {return id_;}
  std::string getInstanceId() {return instance_id_;}

private:
  friend class ::BondSM;
  rclcpp::TimerBase::SharedPtr connect_timer_;
  rclcpp::TimerBase::SharedPtr disconnect_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr publishing_timer_;
  rclcpp::TimerBase::SharedPtr deadpublishing_timer_;
  rclcpp::Node::SharedPtr nh_;
  std::unique_ptr<BondSM> bondsm_;
  BondSMContext sm_;

  std::string topic_;
  std::string id_;
  std::string instance_id_;
  std::string sister_instance_id_;
  std::function<void(void)> on_broken_;
  std::function<void(void)> on_formed_;
  bool sisterDiedFirst_;
  bool started_;
  bool connect_timer_reset_flag_;
  bool disconnect_timer_reset_flag_;
  bool heartbeat_timer_reset_flag_;
  bool deadpublishing_timer_reset_flag_;
  std::mutex mutex_;
  std::condition_variable condition_;

  double connect_timeout_;
  double heartbeat_timeout_;
  double disconnect_timeout_;
  double heartbeat_period_;
  double dead_publish_period_;

  rclcpp::Subscription<bond::msg::Status>::SharedPtr sub_;
  rclcpp::Publisher<bond::msg::Status>::SharedPtr pub_;

  void onConnectTimeout();
  void onHeartbeatTimeout();
  void onDisconnectTimeout();

  void bondStatusCB(const bond::msg::Status::ConstSharedPtr msg);

  void doPublishing();
  void publishStatus(bool active);

  std::vector<std::function<void(void)>> pending_callbacks_;
  void flushPendingCallbacks();
};

}  // namespace bond


// Internal use only
struct BondSM
{
  explicit BondSM(bond::Bond * b_)
  : b(b_) {}
  void Connected();
  void SisterDied();
  void Death();
  void Heartbeat();
  void StartDying();

private:
  bond::Bond * b;
};

#endif  // BONDCPP__BOND_HPP_
