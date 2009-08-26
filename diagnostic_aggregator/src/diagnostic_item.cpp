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

#include <diagnostic_aggregator/diagnostic_item.h>

using namespace diagnostic_item;
using namespace std;

DiagnosticItem::DiagnosticItem(const diagnostic_msgs::DiagnosticStatus *status)
{
  checked_ = false;
  level_ = status->level;
  name_ = status->name;
  message_ = status->message;
  hw_id_ = status->hardware_id;
  values_ = status->values; // Copy?

  // Replace "/" with "" in name to output
  output_name_ = name_;
  string slash_str = "/";
  string::size_type pos = 0;
  while ((pos = output_name_.find(slash_str, pos)) != string::npos)
  {
    output_name_.replace( pos, slash_str.size(), " ");
    pos++;
  }

}

DiagnosticItem::~DiagnosticItem() {}

void DiagnosticItem::update(const diagnostic_msgs::DiagnosticStatus *status)
{
  if (name_ != status->name)
    ROS_ERROR("Incorrect name when updating DiagnosticItem. Expected %s, got %s", name_.c_str(), status->name.c_str());


  level_ = status->level;
  message_ = status->message;
  hw_id_ = status->hardware_id;
  values_ = status->values; // Copy?
}

diagnostic_msgs::DiagnosticStatus *DiagnosticItem::toStatusMsg()
{
  checked_ = true;

  diagnostic_msgs::DiagnosticStatus *status = new diagnostic_msgs::DiagnosticStatus();
  status->name = output_name_;
  status->level = level_;
  status->message = message_;
  status->hardware_id = hw_id_;
  status->values = values_;

  return status;
}

diagnostic_msgs::DiagnosticStatus *DiagnosticItem::toStatusMsg(std::string prefix, bool stale)
{
  checked_ = true;

  diagnostic_msgs::DiagnosticStatus *status = new diagnostic_msgs::DiagnosticStatus();
  ///\todo Check original name to make sure no "/" characters

  status->name = prefix + "/" + output_name_;
  status->level = level_;
  status->message = message_;
  status->hardware_id = hw_id_;
  status->values = values_;

  if (stale)
    status->level = 3;

  return status;
}


int8_t DiagnosticItem::getLevel() { return level_; }

string DiagnosticItem::getMessage() { return message_; }
string DiagnosticItem::getName() { return name_; }

bool DiagnosticItem::hasChecked() {return checked_;}
  
