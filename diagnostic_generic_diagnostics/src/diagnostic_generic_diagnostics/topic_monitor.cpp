#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include "diagnostic_updater/publisher.h"

namespace diagnostic_generic_diagnostics
{
struct TopicStatusParam
{
  TopicStatusParam()
    : topic(""), hardware_id(""), custom_msg(""), headerless(false), fparam(&this->freq.min, &this->freq.max), tparam()
  {
    freq.max = 0.0;
    freq.min = 0.0;
  }
  std::string topic;
  std::string hardware_id;
  std::string custom_msg;
  bool        headerless;
  struct
  {
    double max;
    double min;
  } freq;
  diagnostic_updater::FrequencyStatusParam fparam;
  diagnostic_updater::TimeStampStatusParam tparam;
};

bool parseTopicStatus(XmlRpc::XmlRpcValue &values, TopicStatusParam &param)
{
  ROS_INFO("parsing param...");
  ROS_INFO("values.getType() %d", values.getType());
  if(values.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    if(values["topic"].getType() == XmlRpc::XmlRpcValue::TypeString)
      param.topic = static_cast<std::string>(values["topic"]);
    else
      return false;  // topic name is required

    if(values["hardware_id"].getType() == XmlRpc::XmlRpcValue::TypeString)
      param.hardware_id = static_cast<std::string>(values["hardware_id"]);
    if(values["custom_msg"].getType() == XmlRpc::XmlRpcValue::TypeString)
      param.custom_msg = static_cast<std::string>(values["custom_msg"]);
    if(values["headerless"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
      param.headerless = static_cast<bool>(values["headerless"]);
    if(values["max_freq"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      param.freq.max = static_cast<double>(values["max_freq"]);
    if(values["min_freq"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      param.freq.min = static_cast<double>(values["min_freq"]);
    if(values["tolerance"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      param.fparam.tolerance_ = static_cast<double>(values["tolerance"]);
    if(values["window_size"].getType() == XmlRpc::XmlRpcValue::TypeInt)
      param.fparam.window_size_ = static_cast<int>(values["window_size"]);
    if(values["max_acceptable"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      param.tparam.max_acceptable_ = static_cast<double>(values["max_acceptable"]);
    if(values["min_acceptable"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      param.tparam.min_acceptable_ = static_cast<double>(values["min_acceptable"]);

    return true;
  }
  return false;
}

class TopicMonitor
{
private:
  ros::NodeHandle              nh_, pnh_;
  std::vector<ros::Subscriber> subs_;

  static void callback(const ros::MessageEvent<topic_tools::ShapeShifter> &           msg,
                       std::shared_ptr<diagnostic_updater::Updater>                   updater,
                       std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> task)
  {
    ROS_INFO_THROTTLE(1, "callback, %s", task->getName().c_str());
    task->tick();
    updater->update();
  }

public:
  TopicMonitor(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
  {
    ROS_INFO("Starting TopicMonitor...");
    XmlRpc::XmlRpcValue topics;
    pnh_.getParam("topics", topics);
    ROS_ASSERT(topics.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(topics.size() > 0);
    for(int i = 0; i < topics.size(); ++i)
    {
      ROS_INFO("Reading %d...", i);
      TopicStatusParam param;
      if(parseTopicStatus(topics[i], param))
      {
        auto updater = std::make_shared<diagnostic_updater::Updater>();
        updater->setHardwareID(param.hardware_id);
        std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> watcher;
        if(param.headerless)
        {
          watcher = std::make_shared<diagnostic_updater::HeaderlessTopicDiagnostic>("Headerless Topic Monitor",
                                                                                    *updater, param.fparam);
        }
        else
        {
          watcher = std::make_shared<diagnostic_updater::TopicDiagnostic>("Topic Monitor", *updater, param.fparam,
                                                                          param.tparam);
        }

        auto sub = nh_.subscribe<topic_tools::ShapeShifter>(
            param.topic, 1, boost::bind(diagnostic_generic_diagnostics::TopicMonitor::callback, _1, updater, watcher));
        subs_.push_back(sub);
      }
    }
  }
};

};  // namespace diagnostic_generic_diagnostics

int main(int argc, char **argv)
{
  ros::init(argc, argv, "topic_monitor");

  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");

  diagnostic_generic_diagnostics::TopicMonitor(nh, pnh);

  ros::spin();
  return 0;
}