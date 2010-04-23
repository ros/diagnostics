///\author Kevin Watts

#ifndef _TEST_DIAGNOSTIC_AGGREGATOR_ANALYZE_NO_MATCH_ANALYZER_H_
#define _TEST_DIAGNOSTIC_AGGREGATOR_ANALYZE_NO_MATCH_ANALYZER_H_

#include <ros/ros.h>
#include <diagnostic_aggregator/analyzer.h>
#include <diagnostic_aggregator/status_item.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <pluginlib/class_list_macros.h>
#include <string>

namespace test_diagnostic_aggregator {

class MatchNoAnalyzeAnalyzer : public diagnostic_aggregator::Analyzer
{
private:
  std::string path_, nice_name_, my_item_name_;

  bool has_initialized_, has_item_data_;

public:
  MatchNoAnalyzeAnalyzer();

  ~MatchNoAnalyzeAnalyzer();

  bool init(const std::string base_name, const ros::NodeHandle &n);

  bool match(const std::string name);

  bool analyze(const boost::shared_ptr<diagnostic_aggregator::StatusItem> item);

  std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > report();

  std::string getPath() const { return path_; }

  std::string getName() const { return nice_name_; }
};

}

#endif //_TEST_DIAGNOSTIC_AGGREGATOR_ANALYZE_NO_MATCH_ANALYZER_H_
