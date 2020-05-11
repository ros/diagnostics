
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <std_msgs/Bool.h>
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/publisher.h"

namespace diagnostic_generic_diagnostics
{
struct BoolStatusParam
{
  BoolStatusParam() : topic(""), hardware_id(""), custom_msg(""), timer_update(false), bparam()
  {
    custom_fields.clear();
  }

  std::string                                  topic;
  std::string                                  hardware_id;
  std::string                                  custom_msg;  // Provision. Currentlly not used.
  std::vector<diagnostic_updater::CustomField> custom_fields;
  bool                                         timer_update;
  diagnostic_updater::BoolStatusParam          bparam;
};

bool parseTopicStatus(XmlRpc::XmlRpcValue &values, BoolStatusParam &param)
{
  ROS_DEBUG("parsing param...");
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

    if(values["custom_fields"].getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      auto fields = values["custom_fields"];
      ROS_DEBUG("parsing field, size of %u...", fields.size());
      for(int i = 0; i < fields.size(); ++i)
      {
        diagnostic_updater::CustomField field;
        field.key   = static_cast<std::string>(fields[i]["key"]);
        field.value = static_cast<std::string>(fields[i]["value"]);
        field.level = static_cast<int>(fields[i]["level"]);
        param.custom_fields.push_back(field);
      }
    }

    if(values["publish_error"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
      param.bparam.publish_error_ = static_cast<bool>(values["publish_error"]);
    if(values["invert"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
      param.bparam.invert_ = static_cast<bool>(values["invert"]);
    if(values["timer_update"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
      param.timer_update = static_cast<bool>(values["timer_update"]);

    return true;
  }
  return false;
}

using UpdaterPtr         = std::shared_ptr<diagnostic_updater::Updater>;
using BoolStatusParamPtr = std::shared_ptr<BoolStatusParam>;
class BoolMonitor
{
private:
  ros::NodeHandle              nh_, pnh_;
  ros::Timer                   timer_;
  std::vector<ros::Subscriber> subs_;
  std::vector<ros::Timer>      timers_;

  void callback(const std_msgs::BoolConstPtr &msg, UpdaterPtr updater,
                std::shared_ptr<diagnostic_updater::BoolDiagnostic> task)
  {
    task->set(msg->data);
    updater->update();
  }

  void timerCallback(const ros::TimerEvent &e, UpdaterPtr updater)
  {
    updater->update();
  }

public:
  BoolMonitor(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
  {
    ROS_INFO("Starting BoolMonitor...");

    XmlRpc::XmlRpcValue topics;
    pnh_.getParam("topics", topics);
    ROS_ASSERT(topics.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(topics.size() > 0);
    for(int i = 0; i < topics.size(); ++i)
    {
      ROS_DEBUG("Reading %dth topic...", i);
      auto param = std::make_shared<BoolStatusParam>();
      if(parseTopicStatus(topics[i], *param))
      {
        auto updater = std::make_shared<diagnostic_updater::Updater>();
        updater->setHardwareID(param->hardware_id);

        auto watcher = std::make_shared<diagnostic_updater::BoolDiagnostic>(param->topic, *updater, param->bparam,
                                                                            param->custom_fields);
        auto sub     = nh_.subscribe<std_msgs::Bool>(
            param->topic, 1,
            boost::bind(&diagnostic_generic_diagnostics::BoolMonitor::callback, this, _1, updater, watcher));

        ROS_INFO("Setup sub for %s", param->topic.c_str());
        subs_.push_back(sub);

        if(param->timer_update)
        {
          auto timer = nh_.createTimer(
              ros::Duration(0.5),
              boost::bind(&diagnostic_generic_diagnostics::BoolMonitor::timerCallback, this, _1, updater));
          timers_.push_back(timer);
        }
      }
    }
  }
};

};  // namespace diagnostic_generic_diagnostics
