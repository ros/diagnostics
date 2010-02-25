#include "test_diagnostic_aggregator/match_no_analyze_analyzer.h"

using namespace diagnostic_aggregator;
using namespace test_diagnostic_aggregator;
using namespace std;

PLUGINLIB_REGISTER_CLASS(MatchNoAnalyzeAnalyzer,
                         test_diagnostic_aggregator::MatchNoAnalyzeAnalyzer,
                         diagnostic_aggregator::Analyzer)

MatchNoAnalyzeAnalyzer::MatchNoAnalyzeAnalyzer() :
  path_(""),
  nice_name_(""), 
  my_item_name_(""),
  has_initialized_(false)
{ }

MatchNoAnalyzeAnalyzer::~MatchNoAnalyzeAnalyzer() { }


bool MatchNoAnalyzeAnalyzer::init(const string base_name, const ros::NodeHandle &n)
{ 
  if (!n.getParam("path", nice_name_))
  {
     ROS_ERROR("No power board name was specified in MatchNoAnalyzeAnalyzer! Power board must be \"Power board 10XX\". Namespace: %s", n.getNamespace().c_str());
     return false;
  }

  // path_ = BASE_NAME/Motors
  if (base_name == "/")
    path_ = base_name + nice_name_;
  else
    path_ = base_name + "/" + nice_name_;

  if (!n.getParam("my_item", my_item_name_))
  {
    ROS_ERROR("No parameter \"my_item\" found. Unable to initialize MatchNoAnalyzeAnalyzer!");
    return false;
  }

  has_initialized_ = true;
  
  return true;
}


bool MatchNoAnalyzeAnalyzer::match(const std::string name)
{
  return has_initialized_ && name == my_item_name_;
}

bool MatchNoAnalyzeAnalyzer::analyze(const boost::shared_ptr<StatusItem> item)
{
  ROS_ASSERT_MSG(item->getName() == my_item_name_, "Asked to analyze item that wasn't mine! My name: %s, item: %s", my_item_name_.c_str(), item->getName().c_str());

  return false;
}

vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > MatchNoAnalyzeAnalyzer::report()
{
  vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > output;

  return output;
}
