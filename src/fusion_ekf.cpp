#include "kalman_filter/fusion_ekf.hpp"

#include <iostream>

#include "Eigen/Dense"
#include "kalman_filter/tools.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF()
{
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0, 0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0, 0, 0.0009, 0, 0, 0, 0.09;

  // laser measurement matrix
  H_laser_ << 1, 0, 0, 0, 0, 1, 0, 0;

  /**
   * Initialize ekf_ members
   */

  // create a 4D state vector, we don't know yet the values of the x state
  ekf_.x_ = VectorXd(4);

  // state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1000;

  // measurement matrix
  ekf_.H_ = MatrixXd(2, 4);
  ekf_.H_ << 1, 0, 0, 0, 0, 1, 0, 0;

  // the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;

  // create a process covariance matrix
  ekf_.Q_ = MatrixXd(4, 4);

  // noise
  noise_ax_ = 9;
  noise_ay_ = 9;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage & measurement_pack)
{
  /**
   * Initialization
   */

  if (!is_initialized_) {
    cout << "EKF: " << endl;
    float px = 0.0;
    float py = 0.0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];

      // φ=atan(py/px)
      px = rho * sin(phi);
      py = -rho * cos(phi);

    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // set the state with the initial location and zero velocity
      px = measurement_pack.raw_measurements_[0];
      py = measurement_pack.raw_measurements_[1];
    }

    ekf_.x_ << px, py, 0, 0;

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  // Get elapsed time between updates dt in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // Update transition matrix F with elapsed time dt in seconds
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // Reduce redundant computations
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  ekf_.Q_ << dt_4 / 4 * noise_ax_, 0, dt_3 / 2 * noise_ax_, 0, 0, dt_4 / 4 * noise_ay_, 0,
    dt_3 / 2 * noise_ay_, dt_3 / 2 * noise_ax_, 0, dt_2 * noise_ax_, 0, 0, dt_3 / 2 * noise_ay_, 0,
    dt_2 * noise_ay_;

  ekf_.Predict();

  /**
   * Update
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.R_ = MatrixXd(3, 3);
    ekf_.R_ = R_radar_;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_, Hj_);
  } else {
    ekf_.R_ = MatrixXd(2, 2);
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
