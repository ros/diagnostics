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

#include <stdlib.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <diagnostic_common_diagnostics/libsensors_chip.h>

#define NAME_BUFFER_SIZE 250

SensorChip::SensorChip(sensors_chip_name const *chip_name) :
  internal_name_(chip_name)
{
  char name_buffer[NAME_BUFFER_SIZE];
  sensors_snprintf_chip_name(name_buffer, NAME_BUFFER_SIZE, internal_name_);
  name_ = name_buffer;

  ROS_DEBUG("Found Sensor: %s", getName().c_str());
  enumerate_features();

  std::stringstream info_msg;
  info_msg << "Found sensor " << getName();
  if( features_.size() > 0 ) {
    info_msg << " with features: ";
    size_t remain = features_.size();
    BOOST_FOREACH(const SensorChipFeaturePtr & feature, features_) {
      remain--;
      info_msg << feature->getName();
      if( remain > 0 ) info_msg << ", ";
    }
  } else {
    info_msg << " with no features?";
  }
  ROS_INFO_STREAM(info_msg.str());
}

void SensorChip::enumerate_features(){
  features_.clear();

  sensors_feature const *feature;
  int number = 0;

  while ((feature = sensors_get_features(internal_name_, &number)) != NULL) {
    sensors_feature_type type = feature->type;
    switch(type){
    case SENSORS_FEATURE_FAN:
      features_.push_back(SensorChipFeaturePtr(new FanSensor(*this, feature)));
      break;
    case SENSORS_FEATURE_TEMP:
      features_.push_back(SensorChipFeaturePtr(new TempSensor(*this, feature)));
      break;
    default:
      features_.push_back(SensorChipFeaturePtr(new OtherSensor(*this, feature)));
      break;
    }
  }
}


SensorChipFeature::SensorChipFeature(const SensorChip& chip,
    sensors_feature const *feature) :
  chip_(chip),
  feature_(feature)
{
  name_ = feature_->name;
  char* label_c_str = sensors_get_label(chip_.internal_name_, feature_);
  if( label_c_str == NULL ) {
    ROS_WARN("Could not get label for %s", name_.c_str());
    label_ = "(None)";
  } else {
    label_ = label_c_str;
    free(label_c_str);
  }
  sensor_name_ = getChipName()+"/"+getName();


  ROS_DEBUG("\tFound Feature: %s(%s)[%d]", getLabel().c_str(),
      getName().c_str(), feature_->type);

  enumerate_subfeatures();
}


void SensorChipFeature::enumerate_subfeatures(){
  sensors_subfeature const *subfeature;
  int number = 0;

  while ((subfeature = sensors_get_all_subfeatures(chip_.internal_name_, feature_, &number)) != NULL) {
    sub_features_.push_back(SensorChipSubFeaturePtr(new SensorChipSubFeature(*this, subfeature)));
  }
}


SensorChipSubFeaturePtr SensorChipFeature::getSubFeatureByType(sensors_subfeature_type type){
  BOOST_FOREACH(SensorChipSubFeaturePtr subfeature, sub_features_){
    if(subfeature->getType()==type)
      return subfeature;
  }
  return SensorChipSubFeaturePtr();
}


SensorChipSubFeature::SensorChipSubFeature(const SensorChipFeature& feature,
    sensors_subfeature const *subfeature) :
  feature_(feature),
  subfeature_(subfeature),
  name_(subfeature->name)
{
  ROS_DEBUG("\t\tFound Sub-Feature: %s[%d] = %f", getName().c_str(),
      subfeature_->type, getValue());
}


double SensorChipSubFeature::getValue(){
  double value;
  if( sensors_get_value(feature_.chip_.internal_name_, subfeature_->number,
        &value) != 0 ) {
    ROS_WARN_STREAM("Failed to get value for " << feature_.getSensorName() <<
        " " << getName());
  }
  return value;
}


void FanSensor::buildStatus(diagnostic_updater::DiagnosticStatusWrapper &stat){
  SensorChipSubFeaturePtr speed = getSubFeatureByType(SENSORS_SUBFEATURE_FAN_INPUT);
  if(speed){
    double speed_val = speed->getValue();
    if(speed_val==0)
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
          "Fan stalled or not connected");
    else
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Fan OK (%.2f RPM)",
          speed_val);
    stat.add("Fan Speed (RPM)", speed_val);
  }
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Fan Input!!!");
}


void TempSensor::buildStatus(diagnostic_updater::DiagnosticStatusWrapper &stat){
  SensorChipSubFeaturePtr temp = getSubFeatureByType(SENSORS_SUBFEATURE_TEMP_INPUT);
  SensorChipSubFeaturePtr max_temp = getSubFeatureByType(SENSORS_SUBFEATURE_TEMP_MAX);
  SensorChipSubFeaturePtr temp_crit = getSubFeatureByType(SENSORS_SUBFEATURE_TEMP_CRIT);
  SensorChipSubFeaturePtr temp_crit_alarm = getSubFeatureByType(SENSORS_SUBFEATURE_TEMP_CRIT_ALARM);
  if(temp){
    double temp_val = temp->getValue();
    stat.add("Temperature (\xB0""C)", temp_val);

    if(max_temp && max_temp->getValue()!=0)
      stat.add("Max Temperature (\xB0""C)", max_temp->getValue());
    if(temp_crit && temp_crit->getValue()!=0)
      stat.add("Temperature Critical (\xB0""C)", temp_crit->getValue());

    if(temp_crit_alarm && temp_crit_alarm->getValue()!=0)
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN,
          "Critical Temp Alarm (%.2f\xB0""C)", temp_val);
    else if(temp_crit && temp_crit->getValue()!=0 &&
        temp_val > temp_crit->getValue())
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN,
          "Critical Temp Alarm (%.2f\xB0""C)", temp_val);
    else if(max_temp && max_temp->getValue()!=0 &&
        temp_val > max_temp->getValue())
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN,
          "Max Temp Alarm (%.2f\xB0""C)", temp_val);
    else
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
          "Temp OK (%.2f\xB0""C)", temp_val);
  }
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "NO TEMP Input!!!");
}


void OtherSensor::buildStatus(diagnostic_updater::DiagnosticStatusWrapper &stat){
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Unkown sensor type");
  BOOST_FOREACH(SensorChipSubFeaturePtr subfeature, sub_features_){
    stat.add(subfeature->getName(), subfeature->getValue());
  }
}
