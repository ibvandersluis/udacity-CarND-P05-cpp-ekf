#include "kalman_filter/kalman_filter.hpp"

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(
  VectorXd & x_in, MatrixXd & P_in, MatrixXd & F_in, MatrixXd & H_in, MatrixXd & R_in,
  MatrixXd & Q_in)
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict()
{
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd & z)
{
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd & z, MatrixXd & Hj)
{
  float p_prime_x = x_[0];
  float p_prime_y = x_[1];
  float v_prime_x = x_[2];
  float v_prime_y = x_[3];

  float rho = sqrt(p_prime_x * p_prime_x + p_prime_y * p_prime_y);
  float phi = atan2(p_prime_y, p_prime_x);
  float rho_dot = (p_prime_x * v_prime_x + p_prime_y * v_prime_y) / rho;

  VectorXd h_x_prime(3);
  h_x_prime << rho, phi, rho_dot;

  VectorXd y = z - h_x_prime;

  // Normalize angle to within [-PI,PI]
  while (abs(y[1]) > M_PI) {
    if (y[1] < 0.0) {
      y[1] += (2 * M_PI);
    } else {
      y[1] -= (2 * M_PI);
    }
  }

  MatrixXd Hjt = Hj.transpose();
  MatrixXd S = Hj * P_ * Hjt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Hjt;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj) * P_;
}
