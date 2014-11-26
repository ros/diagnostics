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
#include <algorithm>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <diagnostic_common_diagnostics/libsensors_chip.h>

#define NAME_BUFFER_SIZE 250

SensorChip::SensorChip(sensors_chip_name const *chip_name,
    const std::vector<std::string> &ignore) :
  internal_name_(chip_name)
{
  char name_buffer[NAME_BUFFER_SIZE];
  sensors_snprintf_chip_name(name_buffer, NAME_BUFFER_SIZE, internal_name_);
  name_ = name_buffer;

  ROS_DEBUG("Found Sensor: %s", getName().c_str());

  // enumerate the features of this sensor
  sensors_feature const *feature;
  int number = 0;

  while ((feature = sensors_get_features(internal_name_, &number)) != NULL) {
    sensors_feature_type type = feature->type;
    SensorChipFeaturePtr feature_ptr;
    switch(type){
    case SENSORS_FEATURE_FAN:
      feature_ptr.reset(new FanSensor(*this, feature));
      break;
    case SENSORS_FEATURE_TEMP:
      feature_ptr.reset(new TempSensor(*this, feature));
      break;
    case SENSORS_FEATURE_IN:
      feature_ptr.reset(new VoltageSensor(*this, feature));
      break;
    default:
      feature_ptr.reset(new OtherSensor(*this, feature));
      break;
    }

    if( std::count(ignore.begin(), ignore.end(),
          feature_ptr->getSensorName()) > 0 ) {
      ROS_INFO_STREAM("Ignoring sensor " << feature_ptr->getSensorName());
    } else {
      features_.push_back(feature_ptr);
    }
  }

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
    SensorChipSubFeaturePtr subfeature_ptr(new SensorChipSubFeature(*this,
          subfeature));
    sub_features_.push_back(subfeature_ptr);
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

void VoltageSensor::buildStatus(diagnostic_updater::DiagnosticStatusWrapper &stat){
  SensorChipSubFeaturePtr voltage = getSubFeatureByType(
      SENSORS_SUBFEATURE_IN_INPUT);
  SensorChipSubFeaturePtr min = getSubFeatureByType(
      SENSORS_SUBFEATURE_IN_MIN);
  SensorChipSubFeaturePtr max = getSubFeatureByType(
      SENSORS_SUBFEATURE_IN_MAX);
  SensorChipSubFeaturePtr lcrit = getSubFeatureByType(
      SENSORS_SUBFEATURE_IN_LCRIT);
  SensorChipSubFeaturePtr crit = getSubFeatureByType(
      SENSORS_SUBFEATURE_IN_CRIT);

  // alarm inputs are nonzero on alarm
  SensorChipSubFeaturePtr alarm = getSubFeatureByType(
      SENSORS_SUBFEATURE_IN_ALARM);
  SensorChipSubFeaturePtr min_alarm = getSubFeatureByType(
      SENSORS_SUBFEATURE_IN_MIN_ALARM);
  SensorChipSubFeaturePtr max_alarm = getSubFeatureByType(
      SENSORS_SUBFEATURE_IN_MAX_ALARM);
  SensorChipSubFeaturePtr lcrit_alarm = getSubFeatureByType(
      SENSORS_SUBFEATURE_IN_LCRIT_ALARM);
  SensorChipSubFeaturePtr crit_alarm = getSubFeatureByType(
      SENSORS_SUBFEATURE_IN_CRIT_ALARM);

  if(voltage){
    double voltage_val = voltage->getValue();
    stat.add("Voltage (V)", voltage_val);

    bool low_warn = false;
    bool low_err = false;
    bool high_warn = false;
    bool high_err = false;

    // max voltage (warning)
    if(max) {
      stat.add("Max Voltage (V)", max->getValue());
      if(max_alarm && max_alarm->getValue()!=0) {
        high_warn = true;
      } else if(voltage_val >= max->getValue()) {
        high_warn = true;
      }
    }

    // min voltage (warning)
    if(min) {
      stat.add("Min Voltage (V)", min->getValue());
      if(min_alarm && min_alarm->getValue()!=0) {
        low_warn = true;
      } else if(voltage_val <= min->getValue()) {
        low_warn = true;
      }
    }

    if(crit) {
      stat.add("Critical Max Voltage (V)", crit->getValue());
      if(crit_alarm && crit_alarm->getValue()!=0) {
        high_err = true;
      } else if(voltage_val >= crit->getValue()) {
        high_err = true;
      }
    }

    if(lcrit) {
      stat.add("Critical Min Voltage (V)", lcrit->getValue());
      if(lcrit_alarm && lcrit_alarm->getValue()!=0) {
        low_err = true;
      } else if(voltage_val <= lcrit->getValue()) {
        low_err = true;
      }
    }

    // check for alarms and set summary
    if(high_err) {
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
          "High Voltage CRITICAL (%.2fV)", voltage_val);
    } else if(low_err) {
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
          "Low Voltage CRITICAL (%.2fV)", voltage_val);
    } else if(high_warn) {
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN,
          "High Voltage ALARM (%.2fV)", voltage_val);
    } else if(low_warn) {
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN,
          "Low Voltage ALARM (%.2fV)", voltage_val);
    } else {
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
          "Voltage OK (%.2fV)", voltage_val);
    }
  } else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
        "NO Voltage Input!!!");
  }
}

void OtherSensor::buildStatus(diagnostic_updater::DiagnosticStatusWrapper &stat){
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Unkown sensor type");
  BOOST_FOREACH(SensorChipSubFeaturePtr subfeature, sub_features_){
    stat.add(subfeature->getName(), subfeature->getValue());
  }
}
