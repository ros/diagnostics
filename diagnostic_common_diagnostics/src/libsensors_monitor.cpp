/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Open Source Robotics Foundation, Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the Open Source Robotics Foundation nor the
 *    names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mitchell Wills
 */

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
void enumerate_sensors(const std::vector<std::string> &ignore){
  sensor_chips_.clear();

  sensors_chip_name const *chip_name;
  int number = 0;
  while ((chip_name = sensors_get_detected_chips(NULL, &number)) != NULL){
    SensorChipPtr sensor(new SensorChip(chip_name, ignore));
    sensor_chips_.push_back(sensor);
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
  ros::NodeHandle pnh("~");
  diagnostic_updater::Updater updater(nh, pnh);
  if(!hostname.empty()) {
    updater.setHardwareID(hostname);
  }
  else {
    updater.setHardwareID("none");
  }

  // Reset the libsensors library
  sensors_cleanup();
  if( sensors_init(NULL) != 0 ) {
    ROS_FATAL("Failed to initialize sensors library");
    return 1;
  }

  std::vector<std::string> ignore_sensors;
  pnh.getParam("ignore_sensors", ignore_sensors);
  enumerate_sensors(ignore_sensors);

  if(sensor_chips_.size() <= 0) {
    ROS_ERROR("No sensors detected");
  }

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

  return 0;
}

