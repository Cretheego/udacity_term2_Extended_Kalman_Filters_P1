#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
	is_initialized_ = false;

	previous_timestamp_ = 0;

	// initializing matrices
	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	H_laser_ = MatrixXd(2, 4);
	Hj_ = MatrixXd(3, 4);

	//measurement covariance matrix - laser
	R_laser_ << 0.0225, 0,
				0, 0.0225;

	//measurement covariance matrix - radar
	R_radar_ << 0.09, 0, 0,
				0, 0.0009, 0,
				0, 0, 0.09;

	/**
	TODO:
	* Finish initializing the FusionEKF.
	* Set the process and measurement noises
	*/
	H_laser_ << 1, 0, 0, 0,
				0, 1, 0, 0;
			  
	//Hj_ = CalculateJacobian(const VectorXd& x_state);
	ekf_.P_ = MatrixXd(4,4);
	ekf_.P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;
			  
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;	
			  
	ekf_.Q_ = MatrixXd(4, 4);
	int N = 0.000;
	ekf_.Q_ << N, 0, N, 0,
			  0, N, 0, N,
			  N, 0, N, 0,
			  0, N, 0, N;	
			  
	noise_ax = 9;
	noise_ay = 9;
} 

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

	MatrixXd zeros(3,4);
	zeros << 0,0,0,0,
			 0,0,0,0,
			 0,0,0,0;
	/*****************************************************************************
	*  Initialization
	****************************************************************************/
	if (!is_initialized_) {
		/**
		TODO:
		  * Initialize the state ekf_.x_ with the first measurement.
		  * Create the covariance matrix.
		  * Remember: you'll need to convert radar from polar to cartesian coordinates.
		*/
		// first measurement
		cout << "EKF: " <<endl;
		ekf_.x_ = VectorXd(4);
		ekf_.x_ << 0, 0, 0, 0;
		

		//cout << "I'm here"<<endl;	  
		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
			/**
			Convert radar from polar to cartesian coordinates and initialize state.
			*/
			//return;
			float ro;
			float phi;
			float ro_dot;
			ro = measurement_pack.raw_measurements_[0];
			phi = measurement_pack.raw_measurements_[1];
			ro_dot = measurement_pack.raw_measurements_[2];
			ekf_.x_(0) = sqrt(ro*ro/(1+tan(phi)*tan(phi)));
			ekf_.x_(1) = ekf_.x_(0) * tan(phi);
			//ekf_.x_(2) = (ro * ro_dot )/ekf_.x_(0);
			Hj_ = tools.CalculateJacobian(ekf_.x_);
			cout<<Hj_<<endl;
			if((Hj_ == zeros))
				return;
			ekf_.H_ = Hj_;
			ekf_.R_ = R_radar_;

		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
			/**
			Initialize state.
			*/
			//return;
			ekf_.x_(0) = measurement_pack.raw_measurements_[0];
			ekf_.x_(1) = measurement_pack.raw_measurements_[1];
			//ekf_.Init(&ekf_.x_, &exk_P, &ekf_.F, &ekf_.H_laser_, &ekf_.R_radar_, &ekf_.Q_);	
			ekf_.R_ = R_laser_;
			ekf_.H_ = H_laser_;
		}
		// done initializing, no need to predict or update
		ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, ekf_.H_, ekf_.R_, ekf_.Q_);	
		previous_timestamp_ = measurement_pack.timestamp_;
		is_initialized_ = true;
		return;
	}

	/*****************************************************************************
	*  Prediction
	****************************************************************************/

	/**
	TODO:
	 * Update the state transition matrix F according to the new elapsed time.
	  - Time is measured in seconds.
	 * Update the process noise covariance matrix.
	 * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
	*/
	//cout<<"Q_"<<ekf_.Q_<<endl;
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;

	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;
	//Modify the F matrix so that the time is integrated
	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;
	
	//set the process covariance matrix Q

	ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			   0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			   dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			   0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
	
	ekf_.Predict();
	

	/*****************************************************************************
	*  Update
	****************************************************************************/

	/**
	TODO:
	 * Use the sensor type to perform the update step.
	 * Update the state and covariance matrices.
	*/

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
	// Radar updates
	//return;
	Hj_ = tools.CalculateJacobian(ekf_.x_);
	if((Hj_ == zeros))
		return;
	ekf_.H_ = Hj_;
	ekf_.R_ = R_radar_;
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
	} else {
	// Laser updates
	//cout<<"H_laser_"<<ekf_.H_<<endl;
	//return;
	ekf_.R_ = R_laser_;
	ekf_.H_ = H_laser_;
	ekf_.Update(measurement_pack.raw_measurements_);
	}

	// print the output
	cout << "x_ = " << ekf_.x_ << endl;
	cout << "P_ = " << ekf_.P_ << endl;
}
