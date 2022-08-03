#include "kalman_filter/tools.hpp"

#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(
  const vector<VectorXd> & estimations, const vector<VectorXd> & ground_truth)
{
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    return rmse;
  }

  // Accumulate squared residuals
  for (int i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // Calculate the mean
  rmse = rmse / estimations.size();

  // Calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd & x_state)
{
  MatrixXd Hj(3, 4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // check division by zero
  if (px + py == 0) return Hj;

  // compute denominators in advance for efficiency
  float d1 = px * px + py * py;
  float d2 = sqrt(d1);
  float d3 = (d1 * d2);

  // compute the Jacobian matrix
  Hj << px / d2, py / d2, 0, 0, -py / d1, px / d1, 0, 0, py * (vx * py - vy * px) / d3,
    px * (vy * px - vx * py) / d3, px / d2, py / d2;

  return Hj;
}

float NormalizeAngle(const float & angle)
{
  float normalized = angle;

  while (abs(normalized) > M_PI) {
    if (normalized < 0.0) {
      normalized += (2 * M_PI);
    } else {
      normalized -= (2 * M_PI);
    }
  }

  return normalized;
}
