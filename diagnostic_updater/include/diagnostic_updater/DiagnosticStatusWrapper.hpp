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

#ifndef DIAGNOSTIC_UPDATER__DIAGNOSTICSTATUSWRAPPER_HPP_
#define DIAGNOSTIC_UPDATER__DIAGNOSTICSTATUSWRAPPER_HPP_

#include <stdarg.h>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace diagnostic_updater {

/**
 *
 * \brief Wrapper for the diagnostic_msgs::msg::DiagnosticStatus message that
 * makes it easier to update.
 *
 * This class handles common string formatting and vector handling issues
 * for filling the diagnostic_msgs::msg::DiagnosticStatus message. It is a
 * subclass of
 * diagnostic_msgs::msg::DiagnosticStatus, so it can be passed directly to
 * diagnostic publish calls.
 *
 */
class DiagnosticStatusWrapper : public diagnostic_msgs::msg::DiagnosticStatus {
public:
  /**
   * \brief Fills out the level and message fields of the DiagnosticStatus.
   *
   * \param lvl Numerical level to assign to this Status (OK, Warn, Err).
   * \param s Descriptive status message.
   */
  void summary(unsigned char lvl, const std::string s) {
    level = lvl;
    message = s;
  }

  /**
   * \brief Merges a level and message with the existing ones.
   *
   * It is sometimes useful to merge two DiagnosticStatus messages. In that
   * case,
   * the key value pairs can be unioned, but the level and summary message
   * have to be merged more intelligently. This function does the merge in
   * an intelligent manner, combining the summary in *this, with the one
   * that is passed in.
   *
   * The combined level is the greater of the two levels to be merged.
   * If both levels are non-zero (not OK), the messages are combined with a
   * semicolon separator. If only one level is zero, and the other is
   * non-zero, the message for the zero level is discarded. If both are
   * zero, the new message is ignored.
   *
   * \param lvl Numerical level to of the merged-in summary.
   * \param s Descriptive status message for the merged-in summary.
   */

  void mergeSummary(unsigned char lvl, const std::string s) {
    if ((lvl > 0) && (level > 0)) {
      if (!message.empty())
        message += "; ";
      message += s;
    } else if (lvl > level) {
      message = s;
    }
    if (lvl > level)
      level = lvl;
  }

  /**
   * \brief Version of mergeSummary that merges in the summary from
   * another DiagnosticStatus.
   *
   * \param src DiagnosticStatus from which to merge the summary.
   */

  void mergeSummary(const diagnostic_msgs::msg::DiagnosticStatus &src) {
    mergeSummary(src.level, src.message);
  }

  /**
   * \brief Formatted version of mergeSummary.
   *
   * This method is identical to mergeSummary, except that the message is
   * an sprintf-style format string.
   *
   * \param lvl Numerical level to of the merged-in summary.
   * \param format Format string for the descriptive status message for the
   * merged-in summary.
   * \param ... Values to be formatted by the format string.
   */

  void mergeSummaryf(unsigned char lvl, const char *format, ...) {
    va_list va;
    char buff[1000];  //  @todo This could be done more elegantly.
    va_start(va, format);
    if (vsnprintf(buff, sizeof(buff), format, va) >= 1000)
      ;  //  ROS_DEBUG("Really long string in DiagnosticStatusWrapper::addf, it
        //  was truncated.");
    std::string value = std::string(buff);
    mergeSummary(lvl, value);
    va_end(va);
  }

  /**
   * \brief Formatted version of summary.
   *
   * This method is identical to summary, except that the message is an
   * sprintf-style format string.
   *
   * \param lvl Numerical level to assign to this Status (OK, Warn, Err).
   * \param s Format string for the descriptive status message.
   * \param ... Values to be formatted by the format string.
   *
   */
  void summaryf(unsigned char lvl, const char *format, ...) {
    va_list va;
    char buff[1000];  // @todo This could be done more elegantly.
    va_start(va, format);
    if (vsnprintf(buff, sizeof(buff), format, va) >= 1000)
      ;  //  ROS_DEBUG("Really long string in DiagnosticStatusWrapper::addf, it
        //  was truncated.");
    std::string value = std::string(buff);
    summary(lvl, value);
    va_end(va);
  }

  /**
   * \brief clears the summary, setting the level to zero and the
   * message to "".
   */
  void clearSummary() { summary(0, ""); }

  /**
   * \brief copies the summary from a DiagnosticStatus message
   *
   * \param src StatusWrapper to copy the summary from.
   */
  void summary(const diagnostic_msgs::msg::DiagnosticStatus &src) {
    summary(src.level, src.message);
  }

  /**
   * \brief Add a key-value pair.
   *
   * This method adds a key-value pair. Any type that has a << stream
   * operator can be passed as the second argument.  Formatting is done
   * using a std::stringstream.
   *
   * \param key Key to be added.  \param value Value to be added.
   */
  template <class T> void add(const std::string &key, const T &val) {
    std::stringstream ss;
    ss << val;
    std::string sval = ss.str();
    add(key, sval);
  }

  /**
   * \brief Add a key-value pair using a format string.
   *
   * This method adds a key-value pair. A format string is used to set the
   * value. The current implementation limits the value to 1000 characters
   * in length.
   */

  void addf(const std::string &key, const char *format,
            ...);  // In practice format will always be a char *

  /**
   * \brief Clear the key-value pairs.
   *
   * The values vector containing the key-value pairs is cleared.
   */

  void clear() { values.clear(); }
};

template <>
inline void DiagnosticStatusWrapper::add<std::string>(const std::string &key,
                                                      const std::string &s) {
  diagnostic_msgs::msg::KeyValue ds;
  ds.key = key;
  ds.value = s;
  values.push_back(ds);
}

//  /\brief For bool, diagnostic value is "True" or "False"
template <>
inline void DiagnosticStatusWrapper::add<bool>(const std::string &key,
                                               const bool &b) {
  diagnostic_msgs::msg::KeyValue ds;
  ds.key = key;
  ds.value = b ? "True" : "False";

  values.push_back(ds);
}

// Need to place addf after DiagnosticStatusWrapper::add<std::string> or
// gcc complains that the specialization occurs after instatiation.
inline void
DiagnosticStatusWrapper::addf(const std::string &key, const char *format,
                              ...)  // In practice format will always be a char *
{
  va_list va;
  char buff[1000];  // @todo This could be done more elegantly.
  va_start(va, format);
  if (vsnprintf(buff, sizeof(buff), format, va) >= 1000)
    ;  //  ROS_DEBUG("Really long string in DiagnosticStatusWrapper::addf, it was
      //  truncated.");

  std::string value = std::string(buff);
  add(key, value);
  va_end(va);
}
}  // namespace diagnostic_updater
#endif  // DIAGNOSTIC_UPDATER__DIAGNOSTICSTATUSWRAPPER_HPP_
