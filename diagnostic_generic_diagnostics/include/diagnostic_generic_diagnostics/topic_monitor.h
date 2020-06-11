
#include <unordered_map>

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/publisher.h"

namespace diagnostic_generic_diagnostics
{
struct TopicStatusParam
{
  TopicStatusParam()
    : topic("")
    , hardware_id("anonymous")
    , custom_msg("")
    , headerless(false)
    , fparam(&this->freq.min, &this->freq.max)
    , tparam()
  {
    freq.max = 0.0;
    freq.min = 0.0;
  }

  std::string                                  topic;
  std::string                                  hardware_id;
  std::string                                  custom_msg;  // Provision. Currentlly not used.
  std::vector<diagnostic_updater::CustomField> custom_fields;
  bool                                         headerless;
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
  ros::NodeHandle                             nh_, pnh_;
  ros::Timer                                  timer_;
  std::vector<ros::Subscriber>                subs_;
  std::unordered_map<std::string, UpdaterPtr> updaters_;
  std::vector<std::string>                    hardware_ids_;
  std::vector<TopicStatusParamPtr>            params_;

  void headerlessTopicCallback(const ros::MessageEvent<topic_tools::ShapeShifter> &           msg,
                               std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> task)
  {
    task->tick();
  }

  void topicCallback(const ros::MessageEvent<topic_tools::ShapeShifter> & msg,
                     std::shared_ptr<diagnostic_updater::TopicDiagnostic> task)
  {
    task->tick(ros::Time::now());
  }

  void timerCallback(const ros::TimerEvent &e)
  {
    static int cb_count = 0;
    ROS_DEBUG_THROTTLE(0.1, "cb invoked: %d", cb_count);
    updaters_[hardware_ids_[cb_count]]->update();
    cb_count = (cb_count + 1) % updaters_.size();
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
      ROS_DEBUG("Reading %dth topic...", i);
      auto param = std::make_shared<TopicStatusParam>();
      if(parseTopicStatus(topics[i], *param))
      {
        params_.push_back(param);

        if(updaters_.find(param->hardware_id) == updaters_.end())
        {
          auto u = std::make_shared<diagnostic_updater::Updater>();
          u->setHardwareID(param->hardware_id);
          updaters_[param->hardware_id] = u;
          hardware_ids_.push_back(param->hardware_id);
        }

        ROS_ASSERT(updaters_.size() > 0);
        auto itr = updaters_.find(param->hardware_id);
        ROS_ASSERT(itr != updaters_.end());
        auto updater = itr->second;

        ros::Subscriber sub;
        if(param->headerless)
        {
          auto watcher = std::make_shared<diagnostic_updater::HeaderlessTopicDiagnostic>(
              param->topic, *updater, param->fparam, param->custom_fields);

          sub = nh_.subscribe<topic_tools::ShapeShifter>(
              param->topic, 1,
              boost::bind(&diagnostic_generic_diagnostics::TopicMonitor::headerlessTopicCallback, this, _1, watcher));
        }
        else
        {
          auto watcher = std::make_shared<diagnostic_updater::TopicDiagnostic>(param->topic, *updater, param->fparam,
                                                                               param->tparam, param->custom_fields);

          sub = nh_.subscribe<topic_tools::ShapeShifter>(
              param->topic, 1,
              boost::bind(&diagnostic_generic_diagnostics::TopicMonitor::topicCallback, this, _1, watcher));
        }

        ROS_INFO("Setup sub for %s", param->topic.c_str());
        subs_.push_back(sub);
      }
    }
    auto num_updaters = updaters_.size();
    ROS_ASSERT(num_updaters > 0);
    timer_ = nh_.createTimer(ros::Duration(0.1 / static_cast<double>(num_updaters)),
                             &diagnostic_generic_diagnostics::TopicMonitor::timerCallback, this);
  }
};

};  // namespace diagnostic_generic_diagnostics
