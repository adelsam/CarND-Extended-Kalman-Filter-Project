#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    if (estimations.size() == 0) {
        cout << "Estimate Vector is zero" << endl;
        return rmse;
    }
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() != ground_truth.size()) {
        cout << "Ground Truth Vector size mismatch" << endl;
        return rmse;
    }

    //accumulate squared residuals
    for(int i=0; i < estimations.size(); ++i){
        // ... your code here
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array()*residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse/estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    MatrixXd Hj(3,4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float divisor = pow(px, 2) + pow(py, 2);
    if (divisor > 0) {
        Hj(0,0) = px/pow(divisor,.5);
        Hj(0,1) = py/pow(divisor,.5);

        Hj(1,0) = -py/divisor;
        Hj(1,1) = px/divisor;

        Hj(2,0) = py*(vx*py-vy*px)/pow(divisor,1.5);
        Hj(2,1) = px*(vy*px-vx*py)/pow(divisor,1.5);

        Hj(2,2) = px/pow(divisor,.5);
        Hj(2,3) = py/pow(divisor,.5);
    } else {
        cout << "ERROR DIV zero in jacobian. " << endl;
    }

    //check division by zero

    //compute the Jacobian matrix

    return Hj;
}
