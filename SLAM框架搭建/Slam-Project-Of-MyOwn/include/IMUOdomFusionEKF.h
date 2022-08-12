
#ifndef __IMU_ODOM_FUSION_EKF_H_
#define __IMU_ODOM_FUSION_EKF_H_

#include <Eigen/Dense>
#include <iostream>
#include <math.h>

#define M_PI       3.14159265358979323846

namespace slam{

namespace sensor{

class EKF
{
public:
	EKF();
	~EKF();
	
	void setL(const float l);
	void setDeltaT( const float dt );
	void setNoiseR( const Eigen::Matrix<float, 4, 4> &R );
	void setNoiseR( const float r1, const float r2, const float r3, const float r4 );
	void setNoiseQ( const Eigen::Matrix<float, 6, 6> &Q );
	void setNoiseQ( const float q1, const float q2, const float q3, 
		      	const float q4, const float q5, const float q6 );
	
	const Eigen::Matrix<float, 6, 1> getStateX();	

	void setPreviousState( const float x, const float y, const float theta, const float v, const float w, const float a );
	void setPreviousCovariance( const float p0, const float p1, const float p2, const float p3, const float p4, const float p5 );
	void setpreviousCovariance( const Eigen::Matrix<float, 6, 6> &p );
	
	void setInputObservation( const float vr, const float vl, const float w, const float a );	

	void predict();
	void update( const float vr, const float vl, const float w, const float a );
	
private: 
	void initParameters();

private:
	float l; // robot width
	float dt; //
	
	Eigen::Matrix<float, 6, 1> X; // state vector
	Eigen::Matrix<float, 6, 1> X_estimate; // state estimate
	Eigen::Matrix<float, 6, 1> X_previous; // state at previous time
	
	Eigen::Matrix<float, 4, 1> Z; // observe vector
	Eigen::Matrix<float, 4, 1> Z_estimate; // observation preditcion	

	Eigen::Matrix<float, 6, 6> F; // Jacobian Matrix of the state transition function
	Eigen::Matrix<float, 6, 6> P; // system covariance matrix
	Eigen::Matrix<float, 6, 6> P_estimate;
	Eigen::Matrix<float, 6, 6> P_previous;	

	Eigen::Matrix<float, 4, 6> H; // observe Matrix
	
	Eigen::Matrix<float, 6, 6> Q; // system noise
	Eigen::Matrix<float, 4, 4> R; // observe noise
	
};

}

}

#endif
