#include <sensors/sensors.h>
#include <string>
#include <vector>
#include <boost/foreach.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <ros/ros.h>
#include <diagnostic_common_diagnostics/libsensors_chip.h>
#include <diagnostic_updater/diagnostic_updater.h>

// All of the enumerated sensor chips
std::vector<SensorChipPtr> sensor_chips_;
// Enumerate all of the sensor chips that exist on the system
void enumerate_sensors(){
  sensor_chips_.clear();

  sensors_chip_name const *chip_name;
  int number = 0;
  while ((chip_name = sensors_get_detected_chips(NULL, &number)) != NULL){
    sensor_chips_.push_back(SensorChipPtr(new SensorChip(chip_name)));
  }
}

int main(int argc, char** argv){
  // Get the hostname of the computer
  char hostname_buf[1024];
  int gethostname_result = gethostname(hostname_buf, sizeof(hostname_buf));
  std::string hostname;
  if(gethostname_result == 0) {
    hostname = hostname_buf;
    ROS_INFO_STREAM("Got system hostname: " << hostname);
  }
  else {
    ROS_WARN("Could not get system hostname: %s", strerror(errno));
  }

  if(!hostname.empty()) {
    std::string hostname_clean = boost::replace_all_copy(hostname, "-", "_");
    ros::init(argc, argv, "sensor_monitor_"+hostname_clean);
  }
  else {
    ros::init(argc, argv, "sensor_monitor");
  }


  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  diagnostic_updater::Updater updater(nh, pnh);
  if(!hostname.empty()) {
    updater.setHardwareID(hostname);
  }
  else {
    updater.setHardwareID("none");
  }

  // Reset the libsensors library
  sensors_cleanup();
  sensors_init(NULL);

  enumerate_sensors();

  // Add each sensor to the diagnostic updater
  BOOST_FOREACH(SensorChipPtr sensor_chip, sensor_chips_){
    BOOST_FOREACH(SensorChipFeaturePtr sensor, sensor_chip->features_){
      updater.add(sensor->getSensorName(), boost::bind(&SensorChipFeature::buildStatus, sensor, _1));
    }
  }


  while(ros::ok()){
    updater.update();
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  sensors_cleanup();
}

