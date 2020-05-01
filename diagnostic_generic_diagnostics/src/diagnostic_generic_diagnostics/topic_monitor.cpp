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

using UpdaterPtr          = std::shared_ptr<diagnostic_updater::Updater>;
using TopicStatusParamPtr = std::shared_ptr<TopicStatusParam>;
class TopicMonitor
{
private:
  ros::NodeHandle                  nh_, pnh_;
  ros::Timer                       timer_;
  std::vector<ros::Subscriber>     subs_;
  std::vector<TopicStatusParamPtr> params_;

  void headerlessTopicCallback(const ros::MessageEvent<topic_tools::ShapeShifter> &           msg,
                               diagnostic_generic_diagnostics::UpdaterPtr                     updater,
                               std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> task)
  {
    updater->update();
    task->tick();
  }

  void topicCallback(const ros::MessageEvent<topic_tools::ShapeShifter> & msg,
                     diagnostic_generic_diagnostics::UpdaterPtr           updater,
                     std::shared_ptr<diagnostic_updater::TopicDiagnostic> task)
  {
    updater->update();
    task->tick(ros::Time::now());
  }

  void timerCallback(const ros::TimerEvent &e)
  {
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
      ROS_INFO("Reading %dth topic...", i);
      auto param = std::make_shared<TopicStatusParam>();
      if(parseTopicStatus(topics[i], *param))
      {
        params_.push_back(param);

        auto updater = std::make_shared<diagnostic_updater::Updater>();
        updater->setHardwareID(param->hardware_id);
        ros::Subscriber sub;
        if(param->headerless)
        {
          auto watcher =
              std::make_shared<diagnostic_updater::HeaderlessTopicDiagnostic>(param->topic, *updater, param->fparam);

          sub = nh_.subscribe<topic_tools::ShapeShifter>(
              param->topic, 1, boost::bind(&headerlessTopicCallback, this, _1, updater, watcher));
        }
        else
        {
          auto watcher = std::make_shared<diagnostic_updater::TopicDiagnostic>(param->topic, *updater, param->fparam,
                                                                               param->tparam);

          sub = nh_.subscribe<topic_tools::ShapeShifter>(param->topic, 1,
                                                         boost::bind(&topicCallback, this, _1, updater, watcher));
        }

        ROS_INFO("Setup sub for %s", param->topic.c_str());
        subs_.push_back(sub);
      }
    }
    timer_ = nh_.createTimer(ros::Duration(0.1), &diagnostic_generic_diagnostics::TopicMonitor::timerCallback, this);
  }
};

};  // namespace diagnostic_generic_diagnostics

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