// Copyright (c) 2009, Willow Garage, Inc.

/*  Software License Agreement (BSD License)
 *
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


#ifndef UPDATE_FUNCTIONS_HPP_
#define UPDATE_FUNCTIONS_HPP_

#include <math.h>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"

// WORKAROUND until FrequencyStatus with sim_time support is available (humble+)
namespace topic_monitor
{

using diagnostic_updater::DiagnosticTask;

struct FrequencyStatusParam
{
  FrequencyStatusParam(
    double * min_freq, double * max_freq,
    double tolerance = 0.1, int window_size = 5)
  : min_freq_(min_freq), max_freq_(max_freq), tolerance_(tolerance),
    window_size_(window_size) {}

  double * min_freq_;

  double * max_freq_;

  double tolerance_;

  int window_size_;
};

class FrequencyStatus : public DiagnosticTask
{
private:
  const FrequencyStatusParam params_;

  int count_;
  std::vector<rclcpp::Time> times_;
  std::vector<int> seq_nums_;
  int hist_indx_;
  std::mutex lock_;
  rclcpp::Logger debug_logger_;
  const rclcpp::Clock::SharedPtr clock_ptr_;

public:
  FrequencyStatus(
    const FrequencyStatusParam & params,
    std::string name,
    const rclcpp::Clock::SharedPtr & clock = std::make_shared<rclcpp::Clock>())
  : DiagnosticTask(name), params_(params), times_(params_.window_size_),
    seq_nums_(params_.window_size_),
    debug_logger_(rclcpp::get_logger("FrequencyStatus_debug_logger")),
    clock_ptr_(clock)
  {
    clear();
  }

  explicit FrequencyStatus(
    const FrequencyStatusParam & params,
    const rclcpp::Clock::SharedPtr & clock = std::make_shared<rclcpp::Clock>())
  : FrequencyStatus(params, "Frequency Status", clock)
  {}

  void clear()
  {
    std::unique_lock<std::mutex> lock(lock_);
    rclcpp::Time curtime = clock_ptr_->now();
    count_ = 0;

    for (int i = 0; i < params_.window_size_; i++) {
      times_[i] = curtime;
      seq_nums_[i] = count_;
    }

    hist_indx_ = 0;
  }

  void tick()
  {
    std::unique_lock<std::mutex> lock(lock_);
    RCLCPP_DEBUG(debug_logger_, "TICK %i", count_);
    count_++;
  }

  virtual void run(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    std::unique_lock<std::mutex> lock(lock_);
    rclcpp::Time curtime = clock_ptr_->now();

    int curseq = count_;
    int events = curseq - seq_nums_[hist_indx_];
    double window = curtime.seconds() - times_[hist_indx_].seconds();
    double freq = events / window;
    seq_nums_[hist_indx_] = curseq;
    times_[hist_indx_] = curtime;
    hist_indx_ = (hist_indx_ + 1) % params_.window_size_;

    if (events == 0) {
      stat.summary(2, "No events recorded.");
    } else if (freq < *params_.min_freq_ * (1 - params_.tolerance_)) {
      stat.summary(1, "Frequency too low.");
    } else if (freq > *params_.max_freq_ * (1 + params_.tolerance_)) {
      stat.summary(1, "Frequency too high.");
    } else {
      stat.summary(0, "Desired frequency met");
    }

    stat.addf("Events in window", "%d", events);
    stat.addf("Events since startup", "%d", count_);
    stat.addf("Duration of window (s)", "%f", window);
    stat.addf("Actual frequency (Hz)", "%f", freq);
    if (*params_.min_freq_ == *params_.max_freq_) {
      stat.addf("Target frequency (Hz)", "%f", *params_.min_freq_);
    }
    if (*params_.min_freq_ > 0) {
      stat.addf(
        "Minimum acceptable frequency (Hz)", "%f",
        *params_.min_freq_ * (1 - params_.tolerance_));
    }
    if (std::isfinite(*params_.max_freq_)) {
      stat.addf(
        "Maximum acceptable frequency (Hz)", "%f",
        *params_.max_freq_ * (1 + params_.tolerance_));
    }
  }
};

struct TimeStampStatusParam
{
  TimeStampStatusParam(
    const double min_acceptable = -1,
    const double max_acceptable = 5)
  : max_acceptable_(max_acceptable), min_acceptable_(min_acceptable) {}

  double max_acceptable_;

  double min_acceptable_;
};

static TimeStampStatusParam DefaultTimeStampStatusParam =
  TimeStampStatusParam();

class TimeStampStatus : public DiagnosticTask
{
private:
  void init()
  {
    early_count_ = 0;
    late_count_ = 0;
    zero_count_ = 0;
    zero_seen_ = false;
    max_delta_ = 0;
    min_delta_ = 0;
    deltas_valid_ = false;
  }

public:
  TimeStampStatus(
    const TimeStampStatusParam & params,
    std::string name,
    const rclcpp::Clock::SharedPtr & clock = std::make_shared<rclcpp::Clock>())
  : DiagnosticTask(name), params_(params), clock_ptr_(clock)
  {
    init();
  }

  explicit TimeStampStatus(
    const TimeStampStatusParam & params,
    const rclcpp::Clock::SharedPtr & clock = std::make_shared<rclcpp::Clock>())
  : DiagnosticTask("Timestamp Status"), params_(params), clock_ptr_(clock)
  {
    init();
  }

  explicit TimeStampStatus(
    const rclcpp::Clock::SharedPtr & clock = std::make_shared<rclcpp::Clock>())
  : DiagnosticTask("Timestamp Status"), clock_ptr_(clock) {init();}

  void tick(double stamp)
  {
    std::unique_lock<std::mutex> lock(lock_);

    if (stamp == 0) {
      zero_seen_ = true;
    } else {
      const double delta = clock_ptr_->now().seconds() - stamp;

      if (!deltas_valid_ || delta > max_delta_) {
        max_delta_ = delta;
      }

      if (!deltas_valid_ || delta < min_delta_) {
        min_delta_ = delta;
      }

      deltas_valid_ = true;
    }
  }

  void tick(const rclcpp::Time t) {tick(t.seconds());}

  virtual void run(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    std::unique_lock<std::mutex> lock(lock_);

    stat.summary(0, "Timestamps are reasonable.");
    if (!deltas_valid_) {
      stat.summary(1, "No data since last update.");
    } else {
      if (min_delta_ < params_.min_acceptable_) {
        stat.summary(2, "Timestamps too far in future seen.");
        early_count_++;
      }

      if (max_delta_ > params_.max_acceptable_) {
        stat.summary(2, "Timestamps too far in past seen.");
        late_count_++;
      }

      if (zero_seen_) {
        stat.summary(2, "Zero timestamp seen.");
        zero_count_++;
      }
    }

    stat.addf("Earliest timestamp delay:", "%f", min_delta_);
    stat.addf("Latest timestamp delay:", "%f", max_delta_);
    stat.addf(
      "Earliest acceptable timestamp delay:", "%f",
      params_.min_acceptable_);
    stat.addf(
      "Latest acceptable timestamp delay:", "%f",
      params_.max_acceptable_);
    stat.add("Late diagnostic update count:", late_count_);
    stat.add("Early diagnostic update count:", early_count_);
    stat.add("Zero seen diagnostic update count:", zero_count_);

    deltas_valid_ = false;
    min_delta_ = 0;
    max_delta_ = 0;
    zero_seen_ = false;
  }

private:
  TimeStampStatusParam params_;
  int early_count_;
  int late_count_;
  int zero_count_;
  bool zero_seen_;
  double max_delta_;
  double min_delta_;
  bool deltas_valid_;
  const rclcpp::Clock::SharedPtr clock_ptr_;
  std::mutex lock_;
};

class Heartbeat : public DiagnosticTask
{
public:
  Heartbeat()
  : DiagnosticTask("Heartbeat") {}

  virtual void run(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    stat.summary(0, "Alive");
  }
};
}   // namespace topic_monitor

#endif  // UPDATE_FUNCTIONS_HPP_
