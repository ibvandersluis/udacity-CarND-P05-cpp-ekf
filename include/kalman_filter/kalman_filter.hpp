#ifndef KALMAN_FILTER__KALMAN_FILTER_HPP_
#define KALMAN_FILTER__KALMAN_FILTER_HPP_

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter
{
public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(
    VectorXd & x_in, MatrixXd & P_in, MatrixXd & F_in, MatrixXd & H_in, MatrixXd & R_in,
    MatrixXd & Q_in);

  /**
   * Predict Predicts the state and the state covariance
   *   using the process model
   */
  void Predict();

  /**
   * Updates the state and
   * @param z The measurement at k+1
   */
  void Update(const VectorXd & z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const VectorXd & z, MatrixXd & Hj);

  // state vector
  VectorXd x_;

  // state covariance matrix
  MatrixXd P_;

  // state transistion matrix
  MatrixXd F_;

  // process covariance matrix
  MatrixXd Q_;

  // measurement matrix
  MatrixXd H_;

  // measurement covariance matrix
  MatrixXd R_;
};

#endif  // KALMAN_FILTER__KALMAN_FILTER_HPP_
