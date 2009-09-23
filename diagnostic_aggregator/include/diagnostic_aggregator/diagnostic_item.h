/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

// Author: Kevin Watts

#ifndef DIAGNOSTIC_ITEM_H
#define DIAGNOSTIC_ITEM_H

#include <map>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

namespace diagnostic_item {

/*!
 *\brief Helper class to hold, store DiagnosticStatus messages
 *
 * The DiagnosticItem class is used by the DiagnosticAggregator to store
 * incoming DiagnosticStatus messages. An item stores whether it has been
 * examined by an analyzer. After it has been converted to a DiagnosticStatus
 * message with the "toStatusMsg()" functions, it has been "checked". This
 * is important for processing the "remainder" of DiagnosticStatus messages,
 * those that haven't been analyzed by other analyzers.
 */
class DiagnosticItem
{
public:
  /*!
   *\brief Constructed from DiagnosticStatus*
   */
  DiagnosticItem(const diagnostic_msgs::DiagnosticStatus *status);
  ~DiagnosticItem();

  /*!
   *\brief Must have same name as originial status or it won't update.
   */
  void update(const diagnostic_msgs::DiagnosticStatus *status);

  /*!
   *\brief Sets hasChecked() to true
   */
  diagnostic_msgs::DiagnosticStatus *toStatusMsg();
  
  /*!
   *\brief Sets hasChecked() to true, prepends "prefix/" to name.
   *
   *\param prefix : Prepended to name
   *\param stale : If true, status level is 3
   */
  diagnostic_msgs::DiagnosticStatus *toStatusMsg(std::string prefix, bool stale);

  /*
   *\brief Returns level of DiagnosticStatus message
   */
  int8_t getLevel();
  
  /*!
   *\brief Message field of DiagnosticStatus 
   */
  std::string getMessage();
  
  /*!
   *\brief Returns name of status
   */
  std::string getName();

  /*!
   *\brief True if item has been converted to DiagnosticStatus
   */
  bool hasChecked();

private:
  bool checked_;

  int8_t level_;
  std::string output_name_; /**< name_ w/o "/" */
  std::string name_;
  std::string message_;
  std::string hw_id_;
  std::vector<diagnostic_msgs::KeyValue> values_;
  
};

}

#endif //DIAGNOSTIC_ITEM_H
