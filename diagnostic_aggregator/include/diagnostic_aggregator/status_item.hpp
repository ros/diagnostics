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
#ifndef DIAGNOSTIC_AGGREGATOR__STATUS_ITEM_HPP_
#define DIAGNOSTIC_AGGREGATOR__STATUS_ITEM_HPP_

#include <algorithm>
#include <chrono>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

// TODO(tfoote replace these terrible macros)
#define ROS_ERROR printf
#define ROS_FATAL printf
#define ROS_WARN printf
#define ROS_INFO printf

namespace diagnostic_aggregator
{

/*!
 *\brief Replace "/" with "" in output name, to avoid confusing robot monitor
 */
inline std::string getOutputName(const std::string item_name)
{
  std::string output_name = item_name;
  std::string slash_str = "/";
  std::string::size_type pos = 0;
  while ((pos = output_name.find(slash_str, pos)) != std::string::npos) {
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
  Level_OK = diagnostic_msgs::msg::DiagnosticStatus::OK,
  Level_Warn = diagnostic_msgs::msg::DiagnosticStatus::WARN,
  Level_Error = diagnostic_msgs::msg::DiagnosticStatus::ERROR,
  Level_Stale = 3
};

/*!
 *\brief Converts in to DiagnosticLevel. Values: [0, 3]
 */
inline DiagnosticLevel valToLevel(const int val)
{
  if (val == diagnostic_msgs::msg::DiagnosticStatus::OK) {
    return Level_OK;
  }
  if (val == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
    return Level_Warn;
  }
  if (val == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
    return Level_Error;
  }
  if (val == 3) {
    return Level_Stale;
  }

  ROS_ERROR("Attempting to convert %d into DiagnosticLevel. Values are: {0: "
    "OK, 1: Warning, 2: Error, 3: Stale}",
    val);
  return Level_Error;
}

/*!
 *\brief Converts int to message {0: 'OK', 1: 'Warning', 2: 'Error', 3: 'Stale'
 *}
 */
inline std::string valToMsg(const int val)
{
  if (val == diagnostic_msgs::msg::DiagnosticStatus::OK) {
    return "OK";
  }
  if (val == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
    return "Warning";
  }
  if (val == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
    return "Error";
  }
  if (val == 3) {
    return "Stale";
  }

  ROS_ERROR("Attempting to convert diagnostic level %d into string. Values "
    "are: {0: \"OK\", 1: \"Warning\", 2: \"Error\", 3: \"Stale\"}",
    val);
  return "Error";
}

/*!
 *\brief Removes redundant prefixes from status name.
 *
 * Useful for cleaning up status names.
 * Ex: "/Hokuyo/Tilt HK/tilt_node: Connection" to "/Hokuyo/Tilt HK/Connection"
 *
 * For multiple values of chaff, users will have to run this command for each
 *value. This function won't work properly if multiple chaff values can be
 *removed. For example, name "prosilica_camera: Frequency" with chaff
 *("prosilica", "prosilica_camera") will become "_camera: Frequency" if
 *"prosilica" is removed first.
 */
inline std::string removeLeadingNameChaff(
  const std::string & input_name,
  const std::string & chaff)
{
  std::string output_name = input_name;

  if (chaff.size() == 0) {
    return output_name;
  }

  // Remove start name from all output names
  // Turns "/PREFIX/base_hokuyo_node: Connection Status" to "/PREFIX/Connection
  // Status"
  std::size_t last_slash = output_name.rfind("/");
  std::string start_of_name =
    output_name.substr(0, last_slash) + std::string("/");

  if (output_name.find(chaff) == last_slash + 1) {
    output_name.replace(last_slash + 1, chaff.size(), "");
  }

  if (output_name.find(":", last_slash) == last_slash + 1) {
    output_name = start_of_name + output_name.substr(last_slash + 2);
  }

  while (output_name.find(" ", last_slash) == last_slash + 1) {
    output_name = start_of_name + output_name.substr(last_slash + 2);
  }

  return output_name;
}

/*!
 *\brief Helper class to hold, store DiagnosticStatus messages
 *
 * The StatusItem class is used by the Aggregator to store incoming
 * DiagnosticStatus messages. Helper messages make it easy to calculate update
 * intervals, and extract key_value pairs.
 */
class StatusItem
{
public:
  /*!
   *\brief Constructed from const DiagnosticStatus*
   */
  explicit StatusItem(const diagnostic_msgs::msg::DiagnosticStatus * status);

  /*!
   *\brief Constructed from string of item name
   */
  StatusItem(
    const std::string item_name, const std::string message = "Missing",
    const DiagnosticLevel level = Level_Stale);

  ~StatusItem();

  /*!
   *\brief Must have same name as original status or it won't update.
   *
   *\return True if update successful, false if error
   */
  bool update(const diagnostic_msgs::msg::DiagnosticStatus * status);

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
  std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>
  toStatusMsg(const std::string & path, const bool stale = false) const;

  /*
   *\brief Returns level of DiagnosticStatus message
   */
  DiagnosticLevel getLevel() const {return level_;}

  /*!
   *\brief Get message field of DiagnosticStatus
   */
  std::string getMessage() const {return message_;}

  /*!
   *\brief Returns name of DiagnosticStatus message
   */
  std::string getName() const {return name_;}

  /*!
   *\brief Returns hardware ID field of DiagnosticStatus message
   */
  std::string getHwId() const {return hw_id_;}

  /*!
   *\brief Returns the time since last update for this item
   */
  /*const ros::Time getLastUpdateTime() const { return update_time_; }*/
  const rclcpp::Time getLastUpdateTime() const {return update_time_;}
  /*!
   *\brief Returns true if item has key in values KeyValues
   *
   *\return True if has key
   */
  bool hasKey(const std::string & key) const
  {
    for (unsigned int i = 0; i < values_.size(); ++i) {
      if (values_[i].key == key) {
        return true;
      }
    }

    return false;
  }

  /*!
   *\brief Returns value for given key, "" if doens't exist
   *
   *\return Value if key present, "" if not
   */
  std::string getValue(const std::string & key) const
  {
    for (unsigned int i = 0; i < values_.size(); ++i) {
      if (values_[i].key == key) {
        return values_[i].value;
      }
    }

    return std::string("");
  }

private:
  rclcpp::Time update_time_;

  DiagnosticLevel level_;
  std::string output_name_; /**< name_ w/o "/" */
  std::string name_;
  std::string message_;
  std::string hw_id_;
  std::vector<diagnostic_msgs::msg::KeyValue> values_;
};

}  // namespace diagnostic_aggregator

#endif  // DIAGNOSTIC_AGGREGATOR__STATUS_ITEM_HPP_
