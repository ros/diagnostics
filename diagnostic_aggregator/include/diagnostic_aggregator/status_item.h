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

/*!
 *\author Kevin Watts 
 */

#ifndef DIAGNOSTIC_STATUS_ITEM_H
#define DIAGNOSTIC_STATUS_ITEM_H

#include <map>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <boost/shared_ptr.hpp>

namespace diagnostic_aggregator {

/*!
 *\brief Replace "/" with "" in output name, to avoid confusing robot monitor
 */
inline std::string getOutputName(const std::string item_name)
{
  std::string output_name = item_name;
  std::string slash_str = "/";
  std::string::size_type pos = 0;
  while ((pos = output_name.find(slash_str, pos)) != std::string::npos)
  {
    output_name.replace(pos, slash_str.size(), " ");
    pos++;
  }

  return output_name;
}

/*!
 *\brief Level of StatusItem. OK, Warn, Error, Stale
 */
enum DiagnosticLevel
{
  Level_OK = diagnostic_msgs::DiagnosticStatus::OK,
  Level_Warn = diagnostic_msgs::DiagnosticStatus::WARN,
  Level_Error = diagnostic_msgs::DiagnosticStatus::ERROR,
  Level_Stale = 3
};

/*!
 *\brief Converts in to DiagnosticLevel. Values: [0, 3]
 */
inline DiagnosticLevel valToLevel(const int val)
{
  if (val == diagnostic_msgs::DiagnosticStatus::OK)
    return Level_OK;
  if (val == diagnostic_msgs::DiagnosticStatus::WARN)
    return Level_Warn;
  if (val == diagnostic_msgs::DiagnosticStatus::ERROR)
    return Level_Error;
  if (val == 3)
    return Level_Stale;
  
  ROS_ERROR("Attempting to convert %d into DiagnosticLevel. Values are: {0: OK, 1: Warning, 2: Error, 3: Stale}", val);
  return Level_Error;
}

/*!
 *\brief Converts int to message {0: 'OK', 1: 'Warning', 2: 'Error', 3: 'Stale' }
 */
inline std::string valToMsg(const int val)
{
  if (val == diagnostic_msgs::DiagnosticStatus::OK)
    return "OK";
  if (val == diagnostic_msgs::DiagnosticStatus::WARN)
    return "Warning";
  if (val == diagnostic_msgs::DiagnosticStatus::ERROR)
    return "Error";
  if (val == 3)
    return "Stale";
  
  ROS_ERROR("Attempting to convert diagnostic level %d into string. Values are: {0: \"OK\", 1: \"Warning\", 2: \"Error\", 3: \"Stale\"}", val);
  return "Error";
}

/*!
 *\brief Removes redundant prefixes from status name.
 *
 * Useful for cleaning up status names.
 * Ex: "/Hokuyo/Tilt HK/tilt_node: Connection" to "/Hokuyo/Tilt HK/Connection"
 *
 * For multiple values of chaff, users will have to run this command for each value.
 * This function won't work properly if multiple chaff values can be removed.
 * For example, name "prosilica_camera: Frequency" with chaff ("prosilica", "prosilica_camera")
 * will become "_camera: Frequency" if "prosilica" is removed first. 
 */
inline std::string removeLeadingNameChaff(const std::string &input_name, const std::string &chaff)
{
  std::string output_name = input_name;

  if (chaff.size() == 0)
    return output_name;

  // Remove start name from all output names
  // Turns "/PREFIX/base_hokuyo_node: Connection Status" to "/PREFIX/Connection Status"
  std::size_t last_slash = output_name.rfind("/");
  std::string start_of_name = output_name.substr(0, last_slash) + std::string("/");

  if (output_name.find(chaff) == last_slash + 1)
    output_name.replace(last_slash + 1, chaff.size(), "");

  if (output_name.find(":", last_slash) == last_slash + 1)
    output_name= start_of_name + output_name.substr(last_slash + 2);

  while (output_name.find(" ", last_slash) == last_slash + 1)
    output_name = start_of_name + output_name.substr(last_slash + 2);

  return output_name;
}

/*!
 *\brief Helper class to hold, store DiagnosticStatus messages
 *
 * The StatusItem class is used by the Aggregator to store incoming 
 * DiagnosticStatus messages. Helper messages make it easy to calculate update
 * intervals, and extract KeyValue pairs.
 */
class StatusItem
{
public:
  /*!
   *\brief Constructed from const DiagnosticStatus*
   */
  StatusItem(const diagnostic_msgs::DiagnosticStatus *status);

   /*!
   *\brief Constructed from string of item name
   */
  StatusItem(const std::string item_name, const std::string message = "Missing", const DiagnosticLevel level = Level_Stale);

  ~StatusItem();

  /*!
   *\brief Must have same name as original status or it won't update.
   * 
   *\return True if update successful, false if error
   */
  bool update(const diagnostic_msgs::DiagnosticStatus *status);

  /*!
   *\brief Prepends "path/" to name, makes item stale if "stale" true.
   *
   * Helper function to convert item back to diagnostic_msgs::DiagnosticStatus
   * pointer. Prepends path. 
   * Example: Item with name "Hokuyo" toStatusMsg("Base Path/My Path", false) 
   * gives "Base Path/My Path/Hokuyo". 
   *
   *\param path : Prepended to name
   *\param stale : If true, status level is 3
   */
  boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> toStatusMsg(const std::string &path, const bool stale = false) const;

  /*
   *\brief Returns level of DiagnosticStatus message
   */
  DiagnosticLevel getLevel() const { return level_; }

  /*!
   *\brief Get message field of DiagnosticStatus 
   */
  std::string getMessage() const { return message_; }

  /*!
   *\brief Returns name of DiagnosticStatus message
   */
  std::string getName() const { return name_; }

  /*!
   *\brief Returns hardware ID field of DiagnosticStatus message
   */
  std::string getHwId() const { return hw_id_; }

  /*!
   *\brief Returns the time since last update for this item
   */
  const ros::Time getLastUpdateTime() const { return update_time_; }

  /*!
   *\brief Returns true if item has key in values KeyValues
   *
   *\return True if has key
   */
  bool hasKey(const std::string &key) const
  {
    for (unsigned int i = 0; i < values_.size(); ++i)
    {
      if (values_[i].key == key)
        return true;
    }

    return false;
  }

  /*!
   *\brief Returns value for given key, "" if doens't exist
   *
   *\return Value if key present, "" if not
   */
  std::string getValue(const std::string &key) const
  {
    for (unsigned int i = 0; i < values_.size(); ++i)
    {
      if (values_[i].key == key)
        return values_[i].value;
    }

    return std::string("");
  }

private:
  ros::Time update_time_;

  DiagnosticLevel level_;
  std::string output_name_; /**< name_ w/o "/" */
  std::string name_;
  std::string message_;
  std::string hw_id_;
  std::vector<diagnostic_msgs::KeyValue> values_;
};

}

#endif //DIAGNOSTIC_STATUS_ITEM_H
