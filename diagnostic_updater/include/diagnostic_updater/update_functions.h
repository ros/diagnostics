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

/// Author: Blaise Gassend

#ifndef __DIAGNOSTIC_STATUS__UPDATE_FUNCTIONS_H__
#define __DIAGNOSTIC_STATUS__UPDATE_FUNCTIONS_H__

#include <diagnostic_updater/diagnostic_updater.h>
#include <math.h>

namespace diagnostic_updater
{
/**
 * \brief A structure that holds the custom field parameters.
 */
struct CustomField
{
  std::string key;
  std::string value;
  int         level;  // OK:1, WARN:2, ERROR:4 and sum of the error level you want to show
};

/**
 * \brief A structure that holds the constructor parameters for the
 * FrequencyStatus class.
 */
struct FrequencyStatusParam
{
  /**
   * \brief Creates a filled-out FrequencyStatusParam.
   */

  FrequencyStatusParam(double *min_freq, double *max_freq, double tolerance = 0.1, int window_size = 5)
    : min_freq_(min_freq), max_freq_(max_freq), tolerance_(tolerance), window_size_(window_size)
  {
  }

  /**
   * \brief Minimum acceptable frequency.
   *
   * A pointer is used so that the value can be updated.
   */

  double *min_freq_;

  /**
   * \brief Maximum acceptable frequency.
   *
   * A pointer is used so that the value can be updated.
   */

  double *max_freq_;

  /**
   * \brief Tolerance with which bounds must be satisfied.
   *
   * Acceptable values are from *min_freq_ * (1 - torelance_) to *max_freq_ *
   * (1 + tolerance_).
   *
   * Common use cases are to set tolerance_ to zero, or to assign the same
   * value to *max_freq_ and min_freq_.
   */

  double tolerance_;

  /**
   * \brief Number of events to consider in the statistics.
   */
  int window_size_;
};

/**
 * \brief A diagnostic task that monitors the frequency of an event.
 *
 * This diagnostic task monitors the frequency of calls to its tick method,
 * and creates corresponding diagnostics. It will report a warning if the frequency is
 * outside acceptable bounds, and report an error if there have been no events in the latest
 * window.
 */

class FrequencyStatus : public DiagnosticTask
{
private:
  const FrequencyStatusParam                   params_;
  std::vector<diagnostic_updater::CustomField> custom_fields_;
  int                                          latest_status_;
  int                                          count_;
  std::vector<ros::Time>                       times_;
  std::vector<int>                             seq_nums_;
  int                                          hist_indx_;
  boost::mutex                                 lock_;

public:
  /**
   * \brief Constructs a FrequencyStatus class with the given parameters.
   */

  FrequencyStatus(const FrequencyStatusParam &params, std::string name)
    : DiagnosticTask(name)
    , params_(params)
    , times_(params_.window_size_)
    , seq_nums_(params_.window_size_)
    , latest_status_(diagnostic_msgs::DiagnosticStatus::ERROR)
  {
    clear();
  }

  FrequencyStatus(const FrequencyStatusParam &params, const std::vector<diagnostic_updater::CustomField> &custom_fields)
    : DiagnosticTask("Frequency Status")
    , params_(params)
    , times_(params_.window_size_)
    , seq_nums_(params_.window_size_)
    , latest_status_(diagnostic_msgs::DiagnosticStatus::ERROR)
  {
    clear();
    custom_fields_ = custom_fields;
    ROS_INFO("constructor with custom fields, size of %lu", custom_fields_.size());
  }

  /**
   * \brief Constructs a FrequencyStatus class with the given parameters.
   *        Uses a default diagnostic task name of "Frequency Status".
   */

  FrequencyStatus(const FrequencyStatusParam &params)
    : DiagnosticTask("Frequency Status")
    , params_(params)
    , times_(params_.window_size_)
    , seq_nums_(params_.window_size_)
    , latest_status_(diagnostic_msgs::DiagnosticStatus::ERROR)
  {
    clear();
  }

  int get_status()
  {
    boost::mutex::scoped_lock lock(lock_);
    return latest_status_;
  }

  /**
   * \brief Resets the statistics.
   */

  void clear()
  {
    boost::mutex::scoped_lock lock(lock_);
    ros::Time                 curtime = ros::Time::now();
    count_                            = 0;

    for(int i = 0; i < params_.window_size_; i++)
    {
      times_[i]    = curtime;
      seq_nums_[i] = count_;
    }

    hist_indx_ = 0;
    custom_fields_.clear();
  }

  /**
   * \brief Signals that an event has occurred.
   */
  void tick()
  {
    boost::mutex::scoped_lock lock(lock_);
    // ROS_DEBUG("TICK %i", count_);
    count_++;
  }

  virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    boost::mutex::scoped_lock lock(lock_);
    ros::Time                 curtime = ros::Time::now();
    int                       curseq  = count_;
    int                       events  = curseq - seq_nums_[hist_indx_];
    double                    window  = (curtime - times_[hist_indx_]).toSec();
    double                    freq    = events / window;
    seq_nums_[hist_indx_]             = curseq;
    times_[hist_indx_]                = curtime;
    hist_indx_                        = (hist_indx_ + 1) % params_.window_size_;

    if(events == 0)
    {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
      stat.summary(latest_status_, "No events recorded.");
    }
    else if(freq < *params_.min_freq_ * (1 - params_.tolerance_))
    {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
      stat.summary(latest_status_, "Frequency too low to continue operation.");
    }
    else if(freq > *params_.max_freq_ * (1 + params_.tolerance_))
    {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
      stat.summary(latest_status_, "Frequency too high to continue operation.");
    }
    else if(freq < *params_.min_freq_)
    {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::WARN;
      stat.summary(latest_status_, "Frequency is lower than desired.");
    }
    else if(freq > *params_.max_freq_)
    {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::WARN;
      stat.summary(latest_status_, "Frequency is higher than desired.");
    }
    else
    {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::OK;
      stat.summary(latest_status_, "Desired frequency met");
    }

    for(const auto &field : custom_fields_)
    {
      const bool disp_ok   = (field.level) % 2 == 1;
      const bool disp_warn = (field.level / 2) % 2 == 1;
      const bool disp_err  = (field.level / 4) % 2 == 1;
      if((latest_status_ == diagnostic_msgs::DiagnosticStatus::OK && disp_ok) ||
         (latest_status_ == diagnostic_msgs::DiagnosticStatus::WARN && disp_warn) ||
         (latest_status_ == diagnostic_msgs::DiagnosticStatus::ERROR && disp_err))
      {
        stat.add(field.key, field.value);
      }
    }

    stat.addf("Events in window", "%d", events);
    stat.addf("Events since startup", "%d", count_);
    stat.addf("Duration of window (s)", "%f", window);
    stat.addf("Actual frequency (Hz)", "%f", freq);
    if(*params_.min_freq_ == *params_.max_freq_)
      stat.addf("Target frequency (Hz)", "%f", *params_.min_freq_);
    if(*params_.min_freq_ > 0)
      stat.addf("Minimum acceptable frequency (Hz)", "%f", *params_.min_freq_ * (1 - params_.tolerance_));

#ifdef _WIN32
    if(isfinite(*params_.max_freq_))
#else
    if(finite(*params_.max_freq_))
#endif
      stat.addf("Maximum acceptable frequency (Hz)", "%f", *params_.max_freq_ * (1 + params_.tolerance_));
  }
};

/**
 * \brief A structure that holds the constructor parameters for the
 * TimeStampStatus class.
 */

struct TimeStampStatusParam
{
  /**
   * \brief Creates a filled-out TimeStampStatusParam.
   */

  TimeStampStatusParam(const double min_acceptable = -1, const double max_acceptable = 5)
    : max_acceptable_(max_acceptable), min_acceptable_(min_acceptable)
  {
  }

  /**
   * \brief Maximum acceptable difference between two timestamps.
   */

  double max_acceptable_;

  /**
   * \brief Minimum acceptable difference between two timestamps.
   */

  double min_acceptable_;
};

/**
 * \brief Default TimeStampStatusParam. This is like calling the
 * constructor with no arguments.
 */

static TimeStampStatusParam DefaultTimeStampStatusParam = TimeStampStatusParam();

/**
 * \brief Diagnostic task to monitor the interval between events.
 *
 * This diagnostic task monitors the difference between consecutive events,
 * and creates corresponding diagnostics. An error occurs if the interval
 * between consecutive events is too large or too small. An error condition
 * will only be reported during a single diagnostic report unless it
 * persists. Tallies of errors are also maintained to keep track of errors
 * in a more persistent way.
 */

class TimeStampStatus : public DiagnosticTask
{
private:
  void init()
  {
    early_count_   = 0;
    late_count_    = 0;
    zero_count_    = 0;
    zero_seen_     = false;
    max_delta_     = 0;
    min_delta_     = 0;
    deltas_valid_  = false;
    latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
  }

public:
  /**
   * \brief Constructs the TimeStampStatus with the given parameters.
   */

  TimeStampStatus(const TimeStampStatusParam &params, std::string name) : DiagnosticTask(name), params_(params)
  {
    init();
  }

  /**
   * \brief Constructs the TimeStampStatus with the given parameters.
   *        Uses a default diagnostic task name of "Timestamp Status".
   */

  TimeStampStatus(const TimeStampStatusParam &params) : DiagnosticTask("Timestamp Status"), params_(params)
  {
    init();
  }

  /**
   * \brief Constructs the TimeStampStatus with the default parameters.
   *        Uses a default diagnostic task name of "Timestamp Status".
   */

  TimeStampStatus() : DiagnosticTask("Timestamp Status")
  {
    init();
  }

  int get_status()
  {
    boost::mutex::scoped_lock lock(lock_);
    return latest_status_;
  }

  /**
   * \brief Signals an event. Timestamp stored as a double.
   *
   * \param stamp The timestamp of the event that will be used in computing
   * intervals.
   */
  void tick(double stamp)
  {
    boost::mutex::scoped_lock lock(lock_);

    if(stamp == 0)
    {
      zero_seen_ = true;
    }
    else
    {
      double delta = ros::Time::now().toSec() - stamp;

      if(!deltas_valid_ || delta > max_delta_)
        max_delta_ = delta;

      if(!deltas_valid_ || delta < min_delta_)
        min_delta_ = delta;

      deltas_valid_ = true;
    }
  }

  /**
   * \brief Signals an event.
   *
   * \param t The timestamp of the event that will be used in computing
   * intervals.
   */

  void tick(const ros::Time t)
  {
    tick(t.toSec());
  }

  virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    boost::mutex::scoped_lock lock(lock_);

    latest_status_ = diagnostic_msgs::DiagnosticStatus::OK;
    stat.summary(latest_status_, "Timestamps are reasonable.");
    if(!deltas_valid_)
    {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
      stat.summary(latest_status_, "No data since last update.");
    }
    else
    {
      if(min_delta_ < params_.min_acceptable_)
      {
        latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
        stat.summary(latest_status_, "Timestamps too far in future seen.");
        early_count_++;
      }

      if(max_delta_ > params_.max_acceptable_)
      {
        latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
        stat.summary(latest_status_, "Timestamps too far in past seen.");
        late_count_++;
      }

      if(zero_seen_)
      {
        latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
        stat.summary(latest_status_, "Zero timestamp seen.");
        zero_count_++;
      }
    }

    stat.addf("Earliest timestamp delay:", "%f", min_delta_);
    stat.addf("Latest timestamp delay:", "%f", max_delta_);
    stat.addf("Earliest acceptable timestamp delay:", "%f", params_.min_acceptable_);
    stat.addf("Latest acceptable timestamp delay:", "%f", params_.max_acceptable_);
    stat.add("Late diagnostic update count:", late_count_);
    stat.add("Early diagnostic update count:", early_count_);
    stat.add("Zero seen diagnostic update count:", zero_count_);

    deltas_valid_ = false;
    min_delta_    = 0;
    max_delta_    = 0;
    zero_seen_    = false;
  }

private:
  TimeStampStatusParam params_;
  int                  early_count_;
  int                  late_count_;
  int                  zero_count_;
  bool                 zero_seen_;
  double               max_delta_;
  double               min_delta_;
  bool                 deltas_valid_;
  boost::mutex         lock_;
  int                  latest_status_;
};

/**
 * \brief Diagnostic task to monitor whether a node is alive
 *
 * This diagnostic task always reports as OK and 'Alive' when it runs
 */

class Heartbeat : public DiagnosticTask
{
public:
  /**
   * \brief Constructs a HeartBeat
   */

  Heartbeat() : DiagnosticTask("Heartbeat")
  {
  }

  virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    stat.summary(0, "Alive");
  }
};

struct BoundStatusParam
{
  BoundStatusParam(int error_upper_bound, int warn_upper_bound, int warn_lower_bound, int error_lower_bound)
    : error_upper_bound_(error_upper_bound)
    , warn_upper_bound_(warn_upper_bound)
    , warn_lower_bound_(warn_lower_bound)
    , error_lower_bound_(error_lower_bound)
  {
    if(error_upper_bound_ < error_lower_bound_)
    {
      ROS_WARN("Error upper bound must be grater than error lower bound. Swap them.");
      int tmp            = error_upper_bound;
      error_upper_bound_ = error_lower_bound_;
      error_lower_bound_ = tmp;
    }
    if(warn_upper_bound_ > error_upper_bound_)
    {
      ROS_WARN("Warn upper bound must be less than error upper bound");
      warn_upper_bound_ = error_upper_bound_;
    }
    if(warn_lower_bound_ < error_lower_bound_)
    {
      ROS_WARN("Warn lower bound must be grater than error lower bound");
      warn_lower_bound_ = error_lower_bound_;
    }
  }

  int error_upper_bound_;
  int warn_upper_bound_;
  int warn_lower_bound_;
  int error_lower_bound_;
};

class BoundStatus : public DiagnosticTask
{
private:
  const BoundStatusParam params_;
  boost::mutex           lock_;
  int                    latest_status_;
  int                    current_value_;

public:
  BoundStatus(const BoundStatusParam &params, std::string name = "Bound Status")
    : DiagnosticTask(name), params_(params), current_value_(0), latest_status_(diagnostic_msgs::DiagnosticStatus::ERROR)
  {
    clear();
  }

  void clear()
  {
    boost::mutex::scoped_lock lock(lock_);
    current_value_ = 0;
  }

  void set(int value)
  {
    boost::mutex::scoped_lock lock(lock_);
    current_value_ = value;
  }

  int get_status()
  {
    boost::mutex::scoped_lock lock(lock_);
    return latest_status_;
  }

  virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    boost::mutex::scoped_lock lock(lock_);

    if(current_value_ > params_.error_upper_bound_)
    {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
      stat.summary(latest_status_, "Current value exceeds upper bound.");
    }
    else if(current_value_ < params_.error_lower_bound_)
    {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
      stat.summary(latest_status_, "Current value falls short of lower bound.");
    }
    else if(current_value_ > params_.warn_upper_bound_)
    {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::WARN;
      stat.summary(latest_status_, "Current value is in upper warning range.");
    }
    else if(current_value_ < params_.warn_lower_bound_)
    {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::WARN;
      stat.summary(latest_status_, "Current value is in lower warning range.");
    }
    else
    {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::OK;
      stat.summary(latest_status_, "Current value is in nominal range.");
    }
  }
};

struct CountStatusParam
{
  CountStatusParam(int warn_threshold, int error_threshold)
    : warn_threshold_(warn_threshold), error_threshold_(error_threshold)
  {
    if(warn_threshold_ > error_threshold_)
    {
      ROS_WARN("Error threshold must be grater than or equal to warn threshold.");
      error_threshold_ = warn_threshold_;
    }
  }

  int warn_threshold_;
  int error_threshold_;
};

class CountStatus : public DiagnosticTask
{
private:
  const CountStatusParam params_;
  boost::mutex           lock_;
  int                    latest_status_;
  int                    count_;

public:
  CountStatus(const CountStatusParam &params, std::string name = "Count Status")
    : DiagnosticTask(name), params_(params), latest_status_(diagnostic_msgs::DiagnosticStatus::ERROR)
  {
    clear();
  }

  void clear()
  {
    boost::mutex::scoped_lock lock(lock_);
    count_ = 0;
  }

  int get_status()
  {
    boost::mutex::scoped_lock lock(lock_);
    return latest_status_;
  }

  /**
   * \brief Signals that an event has occurred.
   */
  void tick()
  {
    boost::mutex::scoped_lock lock(lock_);
    count_++;
  }

  virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    boost::mutex::scoped_lock lock(lock_);
    if(count_ >= params_.error_threshold_)
    {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
      stat.summary(latest_status_, "Count exceeds error limit.");
    }
    else if(count_ >= params_.warn_threshold_)
    {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::WARN;
      stat.summary(latest_status_, "Count exceeds warn limit.");
    }
    else
    {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::OK;
      stat.summary(latest_status_, "Count is in nominal range.");
    }
    stat.add("Count:", count_);
    stat.add("Warn Threshold:", params_.warn_threshold_);
    stat.add("Error Threshold:", params_.error_threshold_);
  }
};

struct BoolStatusParam
{
  BoolStatusParam(bool publish_error = true) : publish_error_(publish_error)
  {
  }

  /**
   * \brief Publish ERROR if true, otherwise publish WARN
   */
  bool publish_error_;
};

class BoolStatus : public DiagnosticTask
{
private:
  boost::mutex          lock_;
  const BoolStatusParam params_;
  int                   latest_status_;
  bool                  is_success_;
  int                   success_count_;
  int                   fail_count_;

public:
  /**
   * \brief Constructs a Bool Checker
   */
  BoolStatus(const BoolStatusParam &params, std::string name = "Bool Status")
    : DiagnosticTask(name)
    , params_(params)
    , is_success_(false)
    , latest_status_(diagnostic_msgs::DiagnosticStatus::ERROR)
  {
    clear();
  }

  void clear()
  {
    boost::mutex::scoped_lock lock(lock_);
    success_count_ = 0;
    fail_count_    = 0;
  }

  void set(bool is_success)
  {
    boost::mutex::scoped_lock lock(lock_);
    is_success_ = is_success;
    if(is_success_)
    {
      success_count_++;
      fail_count_ = 0;
    }
    else
    {
      success_count_ = 0;
      fail_count_++;
    }
  }

  int get_status()
  {
    boost::mutex::scoped_lock lock(lock_);
    return latest_status_;
  }

  virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    boost::mutex::scoped_lock lock(lock_);
    if(is_success_)
    {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::OK;
      stat.summary(latest_status_, "Latest process was successfully completed.");
      stat.add("Successfull update count:", success_count_);
    }
    else
    {
      if(params_.publish_error_)
      {
        latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
        stat.summary(latest_status_, "Latest process failed.");
      }
      else
      {
        latest_status_ = diagnostic_msgs::DiagnosticStatus::WARN;
        stat.summary(latest_status_, "Latest process failed.");
      }
      stat.add("Failed update count:", fail_count_);
    }
  }
};
};  // namespace diagnostic_updater

#endif
