#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    /**
    TODO:
      * Calculate the RMSE here.
    */
    VectorXd rmse(4);
	VectorXd residuals;
	rmse << 0,0,0,0;

    // TODO: YOUR CODE HERE

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here
    if(estimations.size() == 0)
    {
        cout<<("Invalidate input estimations")<<endl;
        return rmse;
    }
    if(estimations.size() != ground_truth.size())
    {
        cout<<("Invalidate input estimations")<<endl;
        return rmse;
    }
        
        
	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
        // ... your code here
        residuals = estimations[i] - ground_truth[i]; 
        residuals = residuals.array()*residuals.array();
        rmse += residuals;
		
	}

	//calculate the mean
	// ... your code here
    rmse = rmse / estimations.size();
	cout << "size"<<estimations.size()<<endl;
	//calculate the squared root
	// ... your code here
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
	cout<<"jacob  "<<Hj<<endl;
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE 
    float c1 = px*px+py*py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);
	
	//check division by zero
	if(fabs(c1) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		// set element of Hj equal 0
		Hj << 0,0,0,0,
			0,0,0,0,
			0,0,0,0;
		return Hj;
	}
	//compute the Jacobian matrix
    Hj << px/c2,py/c2,0,0,
        -py/c1,px/c1,0,0,
        py*(vx*py-vy*px)/c3,px*(vy*px-vx*py)/c3,px/c2,py/c2;
	return Hj;
}
