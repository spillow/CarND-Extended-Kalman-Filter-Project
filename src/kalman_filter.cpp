#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  MatrixXd Ht = H_.transpose();

  // Normal linear transformation for lidar
  std::cout << "H = " << H_ << std::endl;
  std::cout << "z = " << z << std::endl;
  VectorXd y = z - H_ * x_;
  std::cout << "y = " << y << std::endl;
  std::cout << "y = " << y << std::endl;
  std::cout << "y = " << y << std::endl;

  std::cout << "bang1" << std::endl;

  MatrixXd S = H_ * P_ * Ht + R_;
  std::cout << "bang2" << std::endl;
  MatrixXd K = P_ * Ht * S.inverse();
  std::cout << "bang3" << std::endl;

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  std::cout << "x = " << x_ << std::endl;
  std::cout << "K = " << K << std::endl;
  std::cout << "y = " << y << std::endl;
  x_ = x_ + K * y;
  std::cout << "bang4" << std::endl;

  P_ = (I - K * H_) * P_;
  std::cout << "bang5" << std::endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  auto h = [](const VectorXd &x)
  {
    float px = x(0);
    float py = x(1);
    float vx = x(2);
    float vy = x(3);

    float denom = sqrt(px*px + py*py);

    if (fabs(denom) < 0.0001)
    {
      std::cout << "h() - Division by Zero" << std::endl;
      return VectorXd(3, 1);
    }

    VectorXd polar = VectorXd(3, 1);
    polar << denom,
             atan2(py, px),
             (px*vx + py*vy) / denom;

    return polar;
  };

  auto normalize = [](float angle)
  {
    int k = (int)(angle / M_2_PI);
    float shift = angle - (float)k * M_2_PI;

    if (shift > M_PI)
      shift -= M_2_PI;
    else if (shift < -M_PI)
      shift += M_2_PI;

    return shift;
  };

  MatrixXd Ht = H_.transpose();

  // Use non-linear mapping to measurement space for radar.
  VectorXd init = z - h(x_);
  VectorXd y(3);
  y(0) = init(0);
  y(1) = normalize(init(1));
  y(2) = init(2);

  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  x_ = x_ + K * y;

  P_ = (I - K * H_) * P_;
}
