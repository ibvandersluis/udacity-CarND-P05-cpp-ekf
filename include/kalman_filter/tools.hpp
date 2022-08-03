#ifndef KALMAN_FILTER__TOOLS_HPP_
#define KALMAN_FILTER__TOOLS_HPP_

#include <vector>

#include "Eigen/Dense"

class Tools
{
public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * @brief 
   * 
   */
  virtual ~Tools();

  /**
   * A helper method to calculate RMSE.
   */
  Eigen::VectorXd CalculateRMSE(
    const std::vector<Eigen::VectorXd> & estimations,
    const std::vector<Eigen::VectorXd> & ground_truth);

  /**
   * A helper method to calculate Jacobians.
   */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd & x_state);

  /**
   * A helper method to normalize angles between -pi and pi
   */
  float NormalizeAngle(const float & angle);
};

#endif  // KALMAN_FILTER__TOOLS_HPP_
