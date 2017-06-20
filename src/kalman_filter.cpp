#include "kalman_filter.h"
#include <iostream>
#include <math.h>

#define PI 3.14159265

using namespace std;
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
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
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

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  VectorXd x_polar = VectorXd(3);
  double rho = pow(pow(x_[0], 2) + pow(x_[1], 2), .5);
  x_polar << rho,
          atan2(x_[1], x_[0]),
          (x_[0] * x_[2] + x_[1] * x_[3]) / rho;

  cout << "x_polar: " << x_polar << endl;

  VectorXd z_pred = x_polar; //polar
  VectorXd y = z - z_pred;
  cout << "y: " << y << endl;
  while (y[1] < -PI) {
    y[1] += 2 * PI;
    cout << "Adding Y" << endl;
  }
  while (y[1] > PI) {
    y[1] -= 2 * PI;
    cout << "Subtracting Y" << endl;
  }


  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;


  VectorXd newEstimate = x_ + (K * y);
  if (isnan(newEstimate[0])) {
    cout << "Update would create nan." << endl;
    cout << "H_: " << H_ << endl;
    cout << "S: " << S << endl;
    cout << "K: " << K << endl;
  }
  //new estimate
  x_ = x_ + (K * y);
  cout << "x_: " << x_ << endl;

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  cout << "P_: " << P_ << endl;

}
