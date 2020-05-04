#include "diagnostic_generic_diagnostics/topic_monitor.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "topic_monitor");

  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");

  diagnostic_generic_diagnostics::TopicMonitor topic_monitor(nh, pnh);

  ROS_INFO("spinning...");
  ros::spin();
  ROS_INFO("exit...");
  return 0;
}
