#include <diagnostic_updater/update_functions.h>

using namespace diagnostic_updater;

void TimeStampStatus::run(DiagnosticStatusWrapper& stat) {
    boost::mutex::scoped_lock lock(lock_);

    using diagnostic_msgs::DiagnosticStatus;

    stat.summary(DiagnosticStatus::OK, "Timestamps are reasonable.");
    if (!deltas_valid_)
    {
      const auto no_data_is_problem = dynamic_cast<SlowTimeStampStatus*>(this) == nullptr;
      const auto status = no_data_is_problem ? DiagnosticStatus::WARN : DiagnosticStatus::OK;
      stat.summary(status, "No data since last update.");

      stat.add("Earliest timestamp delay", "No data");
      stat.add("Latest timestamp delay", "No data");
    }
    else
    {
      if (min_delta_ < params_.min_acceptable_)
      {
	stat.summary(DiagnosticStatus::ERROR, "Timestamps too far in future seen.");
	early_count_++;
      }
    
      if (max_delta_ > params_.max_acceptable_)
      {
	stat.summary(DiagnosticStatus::ERROR, "Timestamps too far in past seen.");
	late_count_++;
      }

      if (zero_seen_)
      {
	stat.summary(DiagnosticStatus::ERROR, "Zero timestamp seen.");
	zero_count_++;
      }

      stat.addf("Earliest timestamp delay", "%f", min_delta_);
      stat.addf("Latest timestamp delay", "%f", max_delta_);
    }

    stat.addf("Earliest acceptable timestamp delay", "%f", params_.min_acceptable_);
    stat.addf("Latest acceptable timestamp delay", "%f", params_.max_acceptable_);
    stat.add("Late diagnostic update count", late_count_);
    stat.add("Early diagnostic update count", early_count_);
    stat.add("Zero seen diagnostic update count", zero_count_);

    deltas_valid_ = false;
    min_delta_ = 0;
    max_delta_ = 0;
    zero_seen_ = false;
}
