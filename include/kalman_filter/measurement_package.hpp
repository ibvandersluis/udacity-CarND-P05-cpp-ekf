#ifndef KALMAN_FILTER__MEASUREMENT_PACKAGE_HPP_
#define KALMAN_FILTER__MEASUREMENT_PACKAGE_HPP_

#include "Eigen/Dense"

class MeasurementPackage
{
public:
  enum SensorType { LASER, RADAR } sensor_type_;

  Eigen::VectorXd raw_measurements_;

  int64_t timestamp_;
};

#endif  // KALMAN_FILTER__MEASUREMENT_PACKAGE_HPP_
