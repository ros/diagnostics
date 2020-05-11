#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "diagnostic_generic_diagnostics/bool_monitor.h"

namespace diagnostic_generic_diagnostics
{
class BoolMonitorNodelet : public nodelet::Nodelet
{
public:
  BoolMonitorNodelet()
  {
  }
  ~BoolMonitorNodelet()
  {
  }

private:
  std::shared_ptr<BoolMonitor> monitor_;
  virtual void                 onInit()
  {
    ros::NodeHandle nh(getNodeHandle());
    ros::NodeHandle pnh(getPrivateNodeHandle());

    monitor_.reset(new BoolMonitor(nh, pnh));
  }
};
};  // namespace diagnostic_generic_diagnostics

PLUGINLIB_EXPORT_CLASS(diagnostic_generic_diagnostics::BoolMonitorNodelet, nodelet::Nodelet)