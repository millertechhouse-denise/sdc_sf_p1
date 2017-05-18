#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) 
{
    //Source: Udacity
	VectorXd rmse(4);
	rmse << 0,0,0,0;
	
	for(int i=0; i<estimations.size(); i++) {
		VectorXd residual = estimations[i] - ground_truth[i];

		residual = residual.array()*residual.array();
		rmse += residual;
	}

	if(estimations.size() > 0)
	{
		rmse  = rmse/estimations.size();
	}

	rmse = rmse.array().sqrt();
	
	return rmse;					 				
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  
  const float EPSILON = 1e-4;
  //Source: Udacity
  MatrixXd jac = MatrixXd::Zero(3,4);
  
  
  
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  if((sqrt(px*px + py*py) < EPSILON))
  {
	  return jac;
  }
  
  jac(0,0) = jac(2,2) = px / (sqrt(px*px + py*py));
  jac(0,1) = jac(2,3) =  py / (sqrt(px*px + py*py));
  jac(1,0) = -py / (px*px + py*py);
  jac(1,1) = px / (px*px + py*py);
  jac(2,0) = py * (vx*py - vy*px) / pow(px*px + py*py, 3.0/2.0);
  jac(2,1) = px * (vy*px - vx*py) / pow(px*px + py*py, 3.0/2.0);
  
  //std::cout << "Jacobian: " << jac << std::endl;  
  
  return jac;
  
  
}
