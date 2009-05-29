#ifndef __FREQUENCY_STATUS_DIAGNOSTIC_H__
#define __FREQUENCY_STATUS_DIAGNOSTIC_H__

#include <diagnostic_updater/diagnostic_updater.h>
#include <math.h>

namespace diagnostic_updater
{
  
class FrequencyStatus : public DiagnosticTask
{
public:
  FrequencyStatus(double &min_freq, double& max_freq, double tolerance = 0.1, int window_size = 5) : 
    DiagnosticTask("Frequency Status"), min_freq_(min_freq), max_freq_(max_freq),
    times_(window_size), seq_nums_(window_size)
  {
    tolerance_ = tolerance;
    window_size_ = window_size;

    clear();
  }

  void clear()
  {
    boost::mutex::scoped_lock lock(lock_);
    ros::Time curtime = ros::Time::now();
    count_ = 0;

    for (int i = 0; i < window_size_; i++)
    {
      times_[i] = curtime;
      seq_nums_[i] = count_;
    }

    hist_indx_ = 0;
  }

  void tick()
  {
    boost::mutex::scoped_lock lock(lock_);
    //ROS_DEBUG("TICK %i", count_);
    count_++;
  }

  void operator()(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    boost::mutex::scoped_lock lock(lock_);
    ros::Time curtime = ros::Time::now();
    int curseq = count_;
    int events = curseq - seq_nums_[hist_indx_];
    double window = (curtime - times_[hist_indx_]).toSec();
    double freq = events / window;
    seq_nums_[hist_indx_] = curseq;
    times_[hist_indx_] = curtime;
    hist_indx_ = (hist_indx_ + 1) % window_size_;

    if (events == 0)
    {
      stat.summary(2, "No events recorded.");
    }
    else if (freq < min_freq_ * (1 - tolerance_))
    {
      stat.summary(2, "Frequency too low.");
    }
    else if (freq > max_freq_ * (1 + tolerance_))
    {
      stat.summary(2, "Frequency too high.");
    }
    else
    {
      stat.summary(0, "Desired frequency met");
    }

    stat.addv("Events in window", events);
    stat.addv("Events since startup", count_);
    stat.addv("Duration of window (s)", window);
    stat.addv("Actual frequency (Hz)", freq);
    if (min_freq_ == max_freq_)
      stat.addv("Target frequency (Hz)", min_freq_);
    if (min_freq_ > 0)
      stat.addv("Minimum acceptable frequency (Hz)", 
          min_freq_ * (1 - tolerance_));
    if (finite(max_freq_))
      stat.addv("Maximum acceptable frequency (Hz)", 
          max_freq_ * (1 + tolerance_));
  }

private:
  double &min_freq_;
  double &max_freq_;

  int count_;
  double tolerance_;
  int window_size_;
  std::vector <ros::Time> times_;
  std::vector <int> seq_nums_;
  int hist_indx_;
  boost::mutex lock_;
};

class TimeStampStatus : public DiagnosticTask
{
public:
  TimeStampStatus(double min_acceptable = -1, double max_acceptable = 5) : 
    DiagnosticTask("Timestamp Status"), 
    early_count_(0), late_count_(0), max_delta_(0), 
    min_delta_(0), deltas_valid_(false), max_acceptable_(max_acceptable),
    min_acceptable_(min_acceptable)
  {}

  void tick(double stamp)
  {
    
    boost::mutex::scoped_lock lock(lock_);
    double delta = ros::Time::now().toSec() - stamp;

    if (!deltas_valid_ || delta > max_delta_)
      max_delta_ = delta;

    if (!deltas_valid_ || delta < min_delta_)
      min_delta_ = delta;

    deltas_valid_ = true;
  }

  void tick(ros::Time t)
  {
    tick(t.toSec());
  }

  void operator()(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    boost::mutex::scoped_lock lock(lock_);
    if (!deltas_valid_)
      stat.summary(1, "No data since last update.");
    else if (min_delta_ < min_acceptable_)
    {
      stat.summary(2, "Timestamps too far in future seen.");
      early_count_++;
    }
    else if (max_delta_ > max_acceptable_)
    {
      stat.summary(2, "Timestamps too far in past seen.");
      late_count_++;
    }
    else
      stat.summary(0, "Timestamps are reasonable.");

    stat.addv("Earliest timestamp delay:", min_delta_);
    stat.addv("Latest timestamp delay:", max_delta_);
    stat.addv("Earliest acceptable timestamp delay:", min_acceptable_);
    stat.addv("Latest acceptable timestamp delay:", max_acceptable_);
    stat.adds("Late diagnostic update count:", late_count_); 
    stat.adds("Early diagnostic update count:", early_count_); 

    deltas_valid_ = false;
    min_delta_ = 0;
    max_delta_ = 0;
  }
private:

  int early_count_;
  int late_count_;
  double max_delta_;
  double min_delta_;
  bool deltas_valid_;
  double max_acceptable_;
  double min_acceptable_;
  boost::mutex lock_;
};

};

#endif
