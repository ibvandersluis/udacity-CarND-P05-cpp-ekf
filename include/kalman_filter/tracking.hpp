#ifndef KALMAN_FILTER__TRACKING_HPP_
#define KALMAN_FILTER__TRACKING_HPP_

#include <fstream>
#include <string>
#include <vector>

#include "kalman_filter/kalman_filter.hpp"
#include "kalman_filter/measurement_package.hpp"

class Tracking
{
public:
  Tracking();
  virtual ~Tracking();
  void ProcessMeasurement(const MeasurementPackage & measurement_pack);
  KalmanFilter kf_;

private:
  bool is_initialized_;
  int64_t previous_timestamp_;

  //acceleration noise components
  float noise_ax;
  float noise_ay;
};

#endif  // KALMAN_FILTER__TRACKING_HPP_
