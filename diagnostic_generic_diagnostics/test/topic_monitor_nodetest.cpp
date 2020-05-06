#include <ros/ros.h>
#include <gtest/gtest.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>

namespace
{
};  // namespace

class TopicMonitorTest : public ::testing::Test
{
public:
  TopicMonitorTest() : nh(""), pnh("")
  {
    pub_hoge = nh.advertise<std_msgs::Header>("/test/hoge", 1);
    pub_fuga = nh.advertise<std_msgs::Bool>("/test/fuga", 1);

    sub = nh.subscribe("/diagnostics", 10, &TopicMonitorTest::callback, this);
  }

  void publish(const int &freq, ros::Publisher &pub)
  {
    if(pub.getTopic() == "/test/hoge")
    {
      std_msgs::Header msg;
      for(int i = 0; i < 2 * freq; ++i)
      {
        msg.stamp = ros::Time::now();
        pub.publish(msg);
        ros::Duration(1.0 / static_cast<double>(freq)).sleep();  // Error range is above 30Hz.
      }
    }
    else
    {
      std_msgs::Bool msg;
      for(int i = 0; i < 2 * freq; ++i)
      {
        pub.publish(msg);
        ros::Duration(1.0 / static_cast<double>(freq)).sleep();  // Error range is above 30Hz.
      }
    }
  }

  void checkKeys(const diagnostic_msgs::DiagnosticStatus &stat)
  {
    for(const auto &value : stat.values)
    {
      for(int i = 0; i < key_found.size(); ++i)
      {
        std::string keystr = "key" + std::to_string(i + 1);
        key_found[i]       = value.key == keystr ? true : key_found[i];
      }
    }
  }

private:
  void callback(const diagnostic_msgs::DiagnosticArrayConstPtr &msgs)
  {
    // std::cerr << "callback invoked: " << msgs->status.size() << std::endl;

    for(const auto &msg : msgs->status)
    {
      // std::cerr << msg.hardware_id << std::endl;
      if(msg.hardware_id == "hoge-hw")
      {
        stat_hoge.hardware_id = msg.hardware_id;
        stat_hoge.level       = msg.level;
        stat_hoge.message     = msg.message;
        stat_hoge.name        = msg.name;
        stat_hoge.values      = msg.values;
      }
      else if(msg.hardware_id == "fuga-hw")
      {
        stat_fuga.hardware_id = msg.hardware_id;
        stat_fuga.level       = msg.level;
        stat_fuga.message     = msg.message;
        stat_fuga.name        = msg.name;
        stat_fuga.values      = msg.values;
      }
      else
      {
      }
    }
  }

protected:
  virtual void SetUp()
  {
    stat_hoge.clear();
    stat_fuga.clear();
    for(auto &e : key_found)
    {
      e = false;
    }
  }

  ros::NodeHandle                             nh, pnh;
  ros::Publisher                              pub_hoge, pub_fuga;
  ros::Subscriber                             sub;
  diagnostic_updater::DiagnosticStatusWrapper stat_hoge, stat_fuga;
  std::array<bool, 7>                         key_found;
};

TEST_F(TopicMonitorTest, subPubInitTest)
{
  ros::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>("/diagnostics", nh, ros::Duration(10));

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
  ros::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>("/diagnostics", nh, ros::Duration(10));

  const int freq = 50;
  publish(freq, pub_hoge);

  EXPECT_EQ(stat_hoge.ERROR, stat_hoge.level);

  checkKeys(stat_hoge);
  EXPECT_FALSE(key_found[1 - 1]);  // "-1" to align array index and disp level
  EXPECT_FALSE(key_found[2 - 1]);
  EXPECT_FALSE(key_found[3 - 1]);
  EXPECT_TRUE(key_found[4 - 1]);
  EXPECT_TRUE(key_found[5 - 1]);
  EXPECT_TRUE(key_found[6 - 1]);
  EXPECT_TRUE(key_found[7 - 1]);
}

TEST_F(TopicMonitorTest, outOfRangeLower)
{
  ros::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>("/diagnostics", nh, ros::Duration(10));

  const int freq = 3;
  publish(freq, pub_hoge);

  EXPECT_EQ(stat_hoge.ERROR, stat_hoge.level);

  checkKeys(stat_hoge);
  EXPECT_FALSE(key_found[1 - 1]);  // "-1" to align array index and disp level
  EXPECT_FALSE(key_found[2 - 1]);
  EXPECT_FALSE(key_found[3 - 1]);
  EXPECT_TRUE(key_found[4 - 1]);
  EXPECT_TRUE(key_found[5 - 1]);
  EXPECT_TRUE(key_found[6 - 1]);
  EXPECT_TRUE(key_found[7 - 1]);
}

TEST_F(TopicMonitorTest, withinWarnRangeUpper)
{
  ros::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>("/diagnostics", nh, ros::Duration(10));

  const int freq = 25;
  publish(freq, pub_hoge);

  EXPECT_EQ(stat_hoge.WARN, stat_hoge.level);

  checkKeys(stat_hoge);
  EXPECT_FALSE(key_found[1 - 1]);  // "-1" to align array index and disp level
  EXPECT_TRUE(key_found[2 - 1]);
  EXPECT_TRUE(key_found[3 - 1]);
  EXPECT_FALSE(key_found[4 - 1]);
  EXPECT_FALSE(key_found[5 - 1]);
  EXPECT_TRUE(key_found[6 - 1]);
  EXPECT_TRUE(key_found[7 - 1]);
}

TEST_F(TopicMonitorTest, withinWarnRangeLower)
{
  ros::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>("/diagnostics", nh, ros::Duration(10));

  const int freq = 7;
  publish(freq, pub_hoge);

  EXPECT_EQ(stat_hoge.WARN, stat_hoge.level);

  checkKeys(stat_hoge);
  EXPECT_FALSE(key_found[1 - 1]);  // "-1" to align array index and disp level
  EXPECT_TRUE(key_found[2 - 1]);
  EXPECT_TRUE(key_found[3 - 1]);
  EXPECT_FALSE(key_found[4 - 1]);
  EXPECT_FALSE(key_found[5 - 1]);
  EXPECT_TRUE(key_found[6 - 1]);
  EXPECT_TRUE(key_found[7 - 1]);
}

TEST_F(TopicMonitorTest, withinOkRange)
{
  ros::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>("/diagnostics", nh, ros::Duration(10));

  const int freq = 15;
  publish(freq, pub_hoge);

  EXPECT_EQ(stat_hoge.OK, stat_hoge.level);

  checkKeys(stat_hoge);
  EXPECT_TRUE(key_found[1 - 1]);  // "-1" to align array index and disp level
  EXPECT_FALSE(key_found[2 - 1]);
  EXPECT_TRUE(key_found[3 - 1]);
  EXPECT_FALSE(key_found[4 - 1]);
  EXPECT_TRUE(key_found[5 - 1]);
  EXPECT_FALSE(key_found[6 - 1]);
  EXPECT_TRUE(key_found[7 - 1]);
}

TEST_F(TopicMonitorTest, outOfRangeUpperHeaderless)
{
  ros::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>("/diagnostics", nh, ros::Duration(10));

  const int freq = 50;
  publish(freq, pub_fuga);

  EXPECT_EQ(stat_fuga.ERROR, stat_fuga.level);

  checkKeys(stat_fuga);
  EXPECT_FALSE(key_found[1 - 1]);  // "-1" to align array index and disp level
  EXPECT_FALSE(key_found[2 - 1]);
  EXPECT_FALSE(key_found[3 - 1]);
  EXPECT_TRUE(key_found[4 - 1]);
  EXPECT_TRUE(key_found[5 - 1]);
  EXPECT_TRUE(key_found[6 - 1]);
  EXPECT_TRUE(key_found[7 - 1]);
}

TEST_F(TopicMonitorTest, outOfRangeLowerHeaderless)
{
  ros::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>("/diagnostics", nh, ros::Duration(10));

  const int freq = 3;
  publish(freq, pub_fuga);

  EXPECT_EQ(stat_fuga.ERROR, stat_fuga.level);

  checkKeys(stat_fuga);
  EXPECT_FALSE(key_found[1 - 1]);  // "-1" to align array index and disp level
  EXPECT_FALSE(key_found[2 - 1]);
  EXPECT_FALSE(key_found[3 - 1]);
  EXPECT_TRUE(key_found[4 - 1]);
  EXPECT_TRUE(key_found[5 - 1]);
  EXPECT_TRUE(key_found[6 - 1]);
  EXPECT_TRUE(key_found[7 - 1]);
}

TEST_F(TopicMonitorTest, withinWarnRangeUpperHeaderless)
{
  ros::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>("/diagnostics", nh, ros::Duration(10));

  const int freq = 35;
  publish(freq, pub_fuga);

  EXPECT_EQ(stat_fuga.WARN, stat_fuga.level);

  checkKeys(stat_fuga);
  EXPECT_FALSE(key_found[1 - 1]);  // "-1" to align array index and disp level
  EXPECT_TRUE(key_found[2 - 1]);
  EXPECT_TRUE(key_found[3 - 1]);
  EXPECT_FALSE(key_found[4 - 1]);
  EXPECT_FALSE(key_found[5 - 1]);
  EXPECT_TRUE(key_found[6 - 1]);
  EXPECT_TRUE(key_found[7 - 1]);
}

TEST_F(TopicMonitorTest, withinWarnRangeLowerHeaderless)
{
  ros::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>("/diagnostics", nh, ros::Duration(10));

  const int freq = 15;
  publish(freq, pub_fuga);

  EXPECT_EQ(stat_fuga.WARN, stat_fuga.level);

  checkKeys(stat_fuga);
  EXPECT_FALSE(key_found[1 - 1]);  // "-1" to align array index and disp level
  EXPECT_TRUE(key_found[2 - 1]);
  EXPECT_TRUE(key_found[3 - 1]);
  EXPECT_FALSE(key_found[4 - 1]);
  EXPECT_FALSE(key_found[5 - 1]);
  EXPECT_TRUE(key_found[6 - 1]);
  EXPECT_TRUE(key_found[7 - 1]);
}

TEST_F(TopicMonitorTest, withinOkRangeHeaderless)
{
  ros::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>("/diagnostics", nh, ros::Duration(10));

  const int freq = 25;
  publish(freq, pub_fuga);

  EXPECT_EQ(stat_fuga.OK, stat_fuga.level);

  checkKeys(stat_fuga);
  EXPECT_TRUE(key_found[1 - 1]);  // "-1" to align array index and disp level
  EXPECT_FALSE(key_found[2 - 1]);
  EXPECT_TRUE(key_found[3 - 1]);
  EXPECT_FALSE(key_found[4 - 1]);
  EXPECT_TRUE(key_found[5 - 1]);
  EXPECT_FALSE(key_found[6 - 1]);
  EXPECT_TRUE(key_found[7 - 1]);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TopicMonitorTest");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
