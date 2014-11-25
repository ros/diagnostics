#ifndef SENSOR_CHIP_H_
#define SENSOR_CHIP_H_

#include <stdlib.h>
#include <sensors/sensors.h>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <diagnostic_updater/diagnostic_updater.h>

class SensorChipSubFeature;
class SensorChipFeature;
class SensorChip;

typedef boost::shared_ptr<SensorChip> SensorChipPtr;
typedef boost::shared_ptr<SensorChipFeature> SensorChipFeaturePtr;
typedef boost::shared_ptr<SensorChipSubFeature> SensorChipSubFeaturePtr;


/**
 * Represents a libsensors sensor chip
 */
class SensorChip{
private:
  std::string name_;
  sensors_chip_name const *internal_name_;
  void enumerate_features();
public:
  SensorChip(sensors_chip_name const *chip_name);
  std::string getName(){return name_;}
  std::vector<SensorChipFeaturePtr> features_;

  friend class SensorChipFeature;
  friend class SensorChipSubFeature;
};


/**
 * Abstract base class for a libsensors sensor chip feature
 */
class SensorChipFeature {
private:
  std::string name_;
  std::string label_;
  std::string sensor_name_;
  SensorChip& chip_;
  sensors_feature const *feature_;
  void enumerate_subfeatures();
public:
  std::vector<SensorChipSubFeaturePtr> sub_features_;
  SensorChipFeature(SensorChip& chip, sensors_feature const *feature);
  SensorChipSubFeaturePtr getSubFeatureByType(sensors_subfeature_type type);

  std::string getName(){return name_;};
  std::string getLabel(){return label_;};
  virtual std::string getSensorName(){return sensor_name_;};
  virtual std::string getSensorLabel(){return getLabel();};
  sensors_feature_type getType(){return feature_->type;};
  std::string getChipName(){return chip_.getName();};

  /**
   * Build a diagnostic status message that represents the current status of the
   * sensor chip feature
   */
  virtual void buildStatus(diagnostic_updater::DiagnosticStatusWrapper &stat) = 0;

  friend class SensorChipSubFeature;
};


/**
 * Represents a libsensors sensor chip sub feature
 */
class SensorChipSubFeature {
private:
  std::string name_;
  SensorChipFeature& feature_;
  sensors_subfeature const *subfeature_;
public:
  SensorChipSubFeature(SensorChipFeature& feature, sensors_subfeature const *subfeature);
  std::string getName();
  sensors_subfeature_type getType();
  double getValue();
};



class FanSensor : public SensorChipFeature{
 public:
  FanSensor(SensorChip& chip, sensors_feature const *feature);
  virtual void buildStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);
};


class TempSensor : public SensorChipFeature{
 public:
  TempSensor(SensorChip& chip, sensors_feature const *feature);
  virtual void buildStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);
};


/**
 * Default case for presenting sensor feature status
 */
class OtherSensor : public SensorChipFeature{
 public:
  OtherSensor(SensorChip& chip, sensors_feature const *feature);
  virtual void buildStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);
};


#endif
