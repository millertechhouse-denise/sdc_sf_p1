#include "kalman_filter.h"
#include <iostream>
#include <stdlib.h> 

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
    * update the state by using Extended Kalman Filter equations
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
    * update the state by using Kalman Filter equations
  */
	const float EPSILON = 1e-3;
	
	float px = x_[0];
	float py = x_[1];
	float vx = x_[2];
	float vy = x_[3];
	
	float ro = sqrt(pow(px,2) + pow(py,2));
	float phi = 0.0;
	float ro_dot = 0.0;
	
	//check that we can perform atan2
	if (fabs(py) > EPSILON && fabs(px) > EPSILON)
	{
		 phi = atan2(py,px); 
	}
	
	//check for divide by zero
	if( fabs(ro) < EPSILON)
	{
		ro_dot = 0.0;
	}
	else
	{
		ro_dot = (px*vx + py*vy) /ro;
	}
	
	VectorXd Hx(3);
	Hx << ro, phi, ro_dot;
	VectorXd y = z - Hx;
	
	//make sure that y(1) is between -pi and pi
	while(y(1) > M_PI)
	{
		y(1) = y(1) - 2 * M_PI;
	}
	
	while(phi < -M_PI)
	{
		y(1) = y(1) + 2 * M_PI;
	}
	
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
