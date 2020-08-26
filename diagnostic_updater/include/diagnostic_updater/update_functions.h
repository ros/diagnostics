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
   * \brief A structure that holds the constructor parameters for the
   * FrequencyStatus class.
   */
  struct FrequencyStatusParam
  {
    /**
     * \brief Creates a filled-out FrequencyStatusParam.
     */

    FrequencyStatusParam(double *min_freq, double *max_freq, double tolerance = 0.1, int window_size = 5) :
      min_freq_(min_freq), max_freq_(max_freq), tolerance_(tolerance), window_size_(window_size)
    {}

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
      const FrequencyStatusParam params_;

      int count_;
      std::vector <ros::Time> times_;
      std::vector <int> seq_nums_;
      int hist_indx_;
      boost::mutex lock_;

    public:
      /**
       * \brief Constructs a FrequencyStatus class with the given parameters.
       */

      FrequencyStatus(const FrequencyStatusParam &params, std::string name) :
        DiagnosticTask(name), params_(params),
        times_(params_.window_size_), seq_nums_(params_.window_size_)
      {
        clear();
      }

      /**
       * \brief Constructs a FrequencyStatus class with the given parameters.
       *        Uses a default diagnostic task name of "Frequency Status".
       */

      FrequencyStatus(const FrequencyStatusParam &params) :
        DiagnosticTask("Frequency Status"), params_(params),
        times_(params_.window_size_), seq_nums_(params_.window_size_)
      {
        clear();
      }

      /**
       * \brief Resets the statistics.
       */

      void clear()
      {
        boost::mutex::scoped_lock lock(lock_);
        ros::Time curtime = ros::Time::now();
        count_ = 0;

        for (int i = 0; i < params_.window_size_; i++)
        {
          times_[i] = curtime;
          seq_nums_[i] = count_;
        }

        hist_indx_ = 0;
      }

      /**
       * \brief Signals that an event has occurred.
       */
      void tick()
      {
        boost::mutex::scoped_lock lock(lock_);
        //ROS_DEBUG("TICK %i", count_);
        count_++;
      }

      virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
      {
        boost::mutex::scoped_lock lock(lock_);
        ros::Time curtime = ros::Time::now();
        int curseq = count_;
        int events = curseq - seq_nums_[hist_indx_];
        double window = (curtime - times_[hist_indx_]).toSec();
        double freq = 0;

        if (window != 0)
        {
            freq = events / window;
        }
        seq_nums_[hist_indx_] = curseq;
        times_[hist_indx_] = curtime;
        hist_indx_ = (hist_indx_ + 1) % params_.window_size_;

        using diagnostic_msgs::DiagnosticStatus;

        if (events == 0)
        {
          stat.summary(DiagnosticStatus::ERROR, "No events recorded.");
        }
        else if (window != 0 && freq < *params_.min_freq_ * (1 - params_.tolerance_))
        {
          stat.summary(DiagnosticStatus::WARN, "Frequency too low.");
        }
        else if (window != 0 && freq > *params_.max_freq_ * (1 + params_.tolerance_))
        {
          stat.summary(DiagnosticStatus::WARN, "Frequency too high.");
        }
        else if (window != 0)
        {
          stat.summary(DiagnosticStatus::OK, "Desired frequency met");
        }

        stat.addf("Events in window", "%d", events);
        stat.addf("Events since startup", "%d", count_);
        stat.addf("Duration of window (s)", "%f", window);
        if (window != 0) {
            stat.addf("Actual frequency (Hz)", "%f",freq);
        }
        if (*params_.min_freq_ == *params_.max_freq_)
          stat.addf("Target frequency (Hz)", "%f",*params_.min_freq_);
        if (*params_.min_freq_ > 0)
          stat.addf("Minimum acceptable frequency (Hz)", "%f",
              *params_.min_freq_ * (1 - params_.tolerance_));

        if (std::isfinite(*params_.max_freq_))
          stat.addf("Maximum acceptable frequency (Hz)", "%f",
              *params_.max_freq_ * (1 + params_.tolerance_));
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

    TimeStampStatusParam(const double min_acceptable = -1, const double max_acceptable = 5) :
      max_acceptable_(max_acceptable), min_acceptable_(min_acceptable)
    {}

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
        early_count_ = 0;
        late_count_ = 0;
        zero_count_ = 0;
        zero_seen_ = false;
        max_delta_ = 0;
        min_delta_ = 0;
        deltas_valid_ = false;
      }

    public:
      /**
       * \brief Constructs the TimeStampStatus with the given parameters.
       */

      TimeStampStatus(const TimeStampStatusParam &params, std::string name) :
        DiagnosticTask(name),
        params_(params)
      {
        init();
      }

      /**
       * \brief Constructs the TimeStampStatus with the given parameters.
       *        Uses a default diagnostic task name of "Timestamp Status".
       */

      TimeStampStatus(const TimeStampStatusParam &params) :
        DiagnosticTask("Timestamp Status"),
        params_(params)
      {
        init();
      }

      /**
       * \brief Constructs the TimeStampStatus with the default parameters.
       *        Uses a default diagnostic task name of "Timestamp Status".
       */

      TimeStampStatus() :
        DiagnosticTask("Timestamp Status")
      {
        init();
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

        if (stamp == 0)
        {
          zero_seen_ = true;
        }
        else
        {
          double delta = ros::Time::now().toSec() - stamp;

          if (!deltas_valid_ || delta > max_delta_)
            max_delta_ = delta;

          if (!deltas_valid_ || delta < min_delta_)
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

      virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

    private:
      TimeStampStatusParam params_;
      int early_count_;
      int late_count_;
      int zero_count_;
      bool zero_seen_;
      double max_delta_;
      double min_delta_;
      bool deltas_valid_;
      boost::mutex lock_;
  };

  /**
   * A TimeStampStatus task that doesn't report periods with no data as a warning. This comes handy if the diagnosed
   * topic has lower frequency than the diagnostic period.
   */
  class SlowTimeStampStatus : public TimeStampStatus
  {
    public:
      SlowTimeStampStatus(const TimeStampStatusParam& params, const std::string& name) : TimeStampStatus(params, name)
      {}

      SlowTimeStampStatus(const TimeStampStatusParam& params) : TimeStampStatus(params)
      {}

      SlowTimeStampStatus()
      {}
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

    Heartbeat() :
      DiagnosticTask("Heartbeat")
    {
    }

    virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
      stat.summary(0, "Alive");
    }
  };
}

#endif
