#ifndef KALMAN_FILTER__FUSION_EKF_HPP_
#define KALMAN_FILTER__FUSION_EKF_HPP_

#include <fstream>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "kalman_filter.hpp"
#include "measurement_package.hpp"
#include "tools.hpp"

class FusionEKF
{
public:
  /**
   * Constructor.
   */
  FusionEKF();

  /**
   * Destructor.
   */
  virtual ~FusionEKF();

  /**
   * Run the whole flow of the Kalman Filter from here.
   */
  void ProcessMeasurement(const MeasurementPackage & measurement_pack);

  /**
   * Kalman Filter update and prediction math lives in here.
   */
  KalmanFilter ekf_;

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;

  float noise_ax_;
  float noise_ay_;
};

#endif  // KALMAN_FILTER__FUSION_EKF_HPP_
