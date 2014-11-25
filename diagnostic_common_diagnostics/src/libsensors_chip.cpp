#include <stdlib.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <diagnostic_common_diagnostics/libsensors_chip.h>

#define NAME_BUFFER_SIZE 50

SensorChip::SensorChip(sensors_chip_name const *chip_name):internal_name_(chip_name){
  char name_buffer[NAME_BUFFER_SIZE];
  sensors_snprintf_chip_name(name_buffer, NAME_BUFFER_SIZE, internal_name_);
  name_ = name_buffer;

  ROS_INFO("Found Sensor: %s", getName().c_str());
  enumerate_features();
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



SensorChipFeature::SensorChipFeature(SensorChip& chip, sensors_feature const *feature):chip_(chip), feature_(feature){
  name_ = feature_->name;
  char* label_c_str = sensors_get_label(chip_.internal_name_, feature_);
  label_ = label_c_str;
  free(label_c_str);
  sensor_name_ = getChipName()+"/"+getName();


  ROS_INFO("\tFound Feature: %s(%s)[%d]", getLabel().c_str(), getName().c_str(), feature_->type);

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




SensorChipSubFeature::SensorChipSubFeature(SensorChipFeature& feature, sensors_subfeature const *subfeature):feature_(feature), subfeature_(subfeature){
  name_ = subfeature_->name;

  ROS_INFO("\t\tFound Sub-Feature: %s[%d] = %f", getName().c_str(), subfeature_->type, getValue());
}
std::string SensorChipSubFeature::getName(){
  return name_;
}
double SensorChipSubFeature::getValue(){
  double value;
  sensors_get_value(feature_.chip_.internal_name_, subfeature_->number, &value);
  return value;
}
sensors_subfeature_type SensorChipSubFeature::getType(){
  return subfeature_->type;
}






FanSensor::FanSensor(SensorChip& chip, sensors_feature const *feature) : SensorChipFeature(chip, feature){}
void FanSensor::buildStatus(diagnostic_updater::DiagnosticStatusWrapper &stat){
  SensorChipSubFeaturePtr speed = getSubFeatureByType(SENSORS_SUBFEATURE_FAN_INPUT);
  if(speed){
    double speed_val = speed->getValue();
    if(speed_val==0)
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Fan stalled or not connected");
    else
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Fan OK (%.2f RPM)", speed_val);
    stat.add("Fan Speed (RPM)", speed_val);
  }
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Fan Input!!!");
}




TempSensor::TempSensor(SensorChip& chip, sensors_feature const *feature) : SensorChipFeature(chip, feature){}
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
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Critical Temp Alarm (%.2f\xB0""C)", temp_val);
    else if(temp_crit && temp_crit->getValue()!=0 && temp_val > temp_crit->getValue())
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Critical Temp Alarm (%.2f\xB0""C)", temp_val);
    else if(max_temp && max_temp->getValue()!=0 && temp_val > max_temp->getValue())
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Max Temp Alarm (%.2f\xB0""C)", temp_val);
    else
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Temp OK (%.2f\xB0""C)", temp_val);
  }
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "NO TEMP Input!!!");
}



OtherSensor::OtherSensor(SensorChip& chip, sensors_feature const *feature) : SensorChipFeature(chip, feature){}
void OtherSensor::buildStatus(diagnostic_updater::DiagnosticStatusWrapper &stat){
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Unkown sensor type");
  BOOST_FOREACH(SensorChipSubFeaturePtr subfeature, sub_features_){
    stat.add(subfeature->getName(), subfeature->getValue());
  }
}
