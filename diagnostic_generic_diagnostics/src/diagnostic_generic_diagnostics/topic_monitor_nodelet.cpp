#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "diagnostic_generic_diagnostics/topic_monitor.h"

namespace diagnostic_generic_diagnostics
{
class TopicMonitorNodelet : public nodelet::Nodelet
{
public:
  TopicMonitorNodelet()
  {
  }
  ~TopicMonitorNodelet()
  {
  }

private:
  std::shared_ptr<TopicMonitor> monitor_;
  virtual void                  onInit()
  {
    ros::NodeHandle nh(getNodeHandle());
    ros::NodeHandle pnh(getPrivateNodeHandle());

    monitor_.reset(new TopicMonitor(nh, pnh));
  }
};
};  // namespace diagnostic_generic_diagnostics

PLUGINLIB_EXPORT_CLASS(diagnostic_generic_diagnostics::TopicMonitorNodelet, nodelet::Nodelet)