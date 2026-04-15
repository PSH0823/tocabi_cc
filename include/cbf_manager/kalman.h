#ifndef KALMAN_H
#define KALMAN_H

#include <Eigen/Dense>

// reference paper: G. Welch, and G. Bishop, "An Introduction to the Kalman Filter" (1995)
// link: https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
class KalmanFilter
{
public:
  KalmanFilter(unsigned int dim_in, unsigned int dim_out, unsigned int dim_state) : l(dim_in), m(dim_out), n(dim_state) {
    using Eigen::MatrixXd;
    using Eigen::VectorXd;

    A = MatrixXd::Identity(n,n);
    B = MatrixXd::Zero(n,l);
    C = MatrixXd::Zero(m,n);

    Q = MatrixXd::Identity(n,n);
    R = MatrixXd::Identity(m,m);
    P = MatrixXd::Identity(n,n);

    K = MatrixXd::Identity(n,m);
    I = MatrixXd::Identity(n,n);

    u = VectorXd::Zero(l);
    q_pred = VectorXd::Zero(n);
    q_est = VectorXd::Zero(n);
    y = VectorXd::Zero(m);
  }

  void predictState() {
    q_pred = A * q_est + B * u;
    P = A * P * A.transpose() + Q;
  }

  void correctState() {
    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    q_est = q_pred + K * (y - C * q_pred);
    P = (I - K * C) * P;
  }

  void updateState() {
    predictState();
    correctState();
  }

public:
  // System matrices:
  Eigen::MatrixXd A;       // State
  Eigen::MatrixXd B;       // Input
  Eigen::MatrixXd C;       // Output

  // Covariance matrices:
  Eigen::MatrixXd Q;       // Process
  Eigen::MatrixXd R;       // Measurement
  Eigen::MatrixXd P;       // Estimate error

  // Kalman gain matrix:
  Eigen::MatrixXd K;

  // Identity matrix
  Eigen::MatrixXd I;

  // Signals:
  Eigen::VectorXd u;       // Input
  Eigen::VectorXd q_pred;  // Predicted state
  Eigen::VectorXd q_est;   // Estimated state
  Eigen::VectorXd y;       // Measurement

private:
  // Dimensions:
  unsigned int l;          // Input
  unsigned int m;          // Output
  unsigned int n;          // State
};

#endif // KALMAN_H