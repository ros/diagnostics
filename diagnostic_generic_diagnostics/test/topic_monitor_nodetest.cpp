#include <ros/ros.h>
#include <gtest/gtest.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>

namespace
{
ros::Publisher                              pub_hoge, pub_fuga;
ros::Subscriber                             sub;
diagnostic_updater::DiagnosticStatusWrapper stat_hoge, stat_fuga;

void callback(const diagnostic_msgs::DiagnosticArrayConstPtr &msgs)
{
  ROS_INFO("callback invoked");
  for(const auto &msg : msgs->status)
    if(msg.hardware_id == "hoge-hw")
    {
      stat_hoge.hardware_id = msg.hardware_id;
      stat_hoge.level       = msg.level;
      stat_hoge.message     = msg.message;
      stat_hoge.name        = msg.name;
    }
    else if(msg.hardware_id == "fuga-hw")
    {
      stat_fuga.hardware_id = msg.hardware_id;
      stat_fuga.level       = msg.level;
      stat_fuga.message     = msg.message;
      stat_fuga.name        = msg.name;
    }
    else
    {
    }
}
};  // namespace

class TopicMonitorTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    stat_hoge.clear();
    stat_fuga.clear();
  }
};

/*
TEST_F(TopicMonitorTest, nhInitTest)
{
  EXPECT_TRUE(ros::master::check());
  EXPECT_TRUE(nh.ok());
  EXPECT_TRUE(pnh.ok());
}
*/

TEST_F(TopicMonitorTest, pubInitTest)
{
  ros::V_string nodes;
  EXPECT_TRUE(ros::master::getNodes(nodes));
  for(const auto &node : nodes)
  {
    EXPECT_STREQ("/topic_monitor", node.c_str());
  }

  ros::master::V_TopicInfo topics;
  EXPECT_TRUE(ros::master::getTopics(topics));
  for(const auto &topic : topics)
  {
    EXPECT_STREQ("/test/hoge", topic.name.c_str());
  }

  EXPECT_STREQ("/test/hoge", pub_hoge.getTopic().c_str());
  EXPECT_EQ(1, pub_hoge.getNumSubscribers());

  EXPECT_STREQ("/test/fuga", pub_fuga.getTopic().c_str());
  EXPECT_EQ(1, pub_fuga.getNumSubscribers());

  EXPECT_STREQ("/diagnostics", sub.getTopic().c_str());
  EXPECT_EQ(1, sub.getNumPublishers());
}

TEST_F(TopicMonitorTest, statInitTest)
{
  EXPECT_STREQ("", stat_hoge.hardware_id.c_str());
  EXPECT_STREQ("", stat_hoge.name.c_str());
  EXPECT_STREQ("", stat_hoge.message.c_str());
  EXPECT_EQ(0, stat_hoge.level);

  EXPECT_STREQ("", stat_fuga.hardware_id.c_str());
  EXPECT_STREQ("", stat_fuga.name.c_str());
  EXPECT_STREQ("", stat_fuga.message.c_str());
  EXPECT_EQ(0, stat_fuga.level);
}

TEST_F(TopicMonitorTest, outOfRangeUpper)
{
  std_msgs::Header msg;
  msg.stamp = ros::Time::now();
  pub_hoge.publish(msg);
  usleep(10000);
  msg.stamp = ros::Time::now();
  pub_hoge.publish(msg);
  usleep(10000);
  msg.stamp = ros::Time::now();
  pub_hoge.publish(msg);

  EXPECT_EQ(stat_hoge.ERROR, stat_hoge.level);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TopicMonitorTest");

  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");
  pub_hoge = nh.advertise<std_msgs::Header>("/test/hoge", 1);
  pub_fuga = nh.advertise<std_msgs::Bool>("/test/fuga", 1);

  sub = nh.subscribe("/diagnostics", 1, callback);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
