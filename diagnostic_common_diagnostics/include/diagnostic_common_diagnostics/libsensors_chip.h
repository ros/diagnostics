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
  sensors_chip_name const * internal_name_;
  void enumerate_features();
public:
  SensorChip(sensors_chip_name const *chip_name);
  const std::string &getName() const {return name_;}
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
  const SensorChip& chip_;
  sensors_feature const *feature_;
  void enumerate_subfeatures();

public:
  std::vector<SensorChipSubFeaturePtr> sub_features_;
  SensorChipFeature(const SensorChip& chip, sensors_feature const *feature);
  SensorChipSubFeaturePtr getSubFeatureByType(sensors_subfeature_type type);

  const std::string &getName() const {return name_;};
  const std::string &getLabel() const {return label_;};
  const std::string &getSensorName() const {return sensor_name_;};
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
  const SensorChipFeature& feature_;
  sensors_subfeature const *subfeature_;
public:
  SensorChipSubFeature(const SensorChipFeature& feature,
      sensors_subfeature const *subfeature);
  const std::string &getName() { return name_; }
  sensors_subfeature_type getType() { return subfeature_->type; }
  double getValue();
};


class FanSensor : public SensorChipFeature{
 public:
  FanSensor(const SensorChip& chip, sensors_feature const *feature) :
    SensorChipFeature(chip, feature) {}
  virtual void buildStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);
};


class TempSensor : public SensorChipFeature{
 public:
  TempSensor(const SensorChip& chip, sensors_feature const *feature) :
    SensorChipFeature(chip, feature) {}
  virtual void buildStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);
};


/**
 * Default case for presenting sensor feature status
 */
class OtherSensor : public SensorChipFeature{
 public:
  OtherSensor(const SensorChip& chip, sensors_feature const *feature) :
    SensorChipFeature(chip, feature) {}
  virtual void buildStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);
};


#endif
