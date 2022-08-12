#include "IMUOdomFusionEKF.h"

namespace slam{

namespace sensor{

EKF::EKF()
{
	initParameters();
}

EKF::~EKF()
{

}

void EKF::initParameters()
{
	l = 10;

	X = Eigen::Matrix<float, 6, 1>::Zero();
	X_estimate = Eigen::Matrix<float, 6, 1>::Zero();
	X_previous = Eigen::Matrix<float, 6, 1>::Zero();	

	Z = Eigen::Matrix<float, 4, 1>::Zero();
	Z_estimate = Eigen::Matrix<float, 4, 1>::Zero();

	P = Eigen::Matrix<float, 6, 6>::Zero();
	P_estimate = Eigen::Matrix<float, 6, 6>::Zero();
	P_previous = Eigen::Matrix<float, 6, 6>::Identity();

	F = Eigen::Matrix<float, 6, 6>::Identity();
	
	Q = Eigen::Matrix<float, 6, 6>::Zero();

	R = Eigen::Matrix<float, 4, 4>::Zero();

	// ---------- Set the Observe Matrix ---------- //
	//  H = | 0, 0, 0, 1,  l/2, 0 |
	//  	| 0, 0, 0, 1, -l/2, 0 |
	//	| 0, 0, 0, 0,   1,  0 |
	//	| 0, 0, 0, 0,   0,  0 |  
	H <<    0, 0, 0, 1, 0.5 * l, 0,
		0, 0, 0, 1, -0.5 * l, 0,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1;
}


void EKF::predict()
{
	// --------- 1. estimate the state according to previous time state -------//
	//	x^(t) = x(t-1) + v(t) * cos( theta(t) ) * dt
	//	y^(t) = y(t-1) + v(t) * sin( theta(t) ) * dt
	//	theta^(t) = theat(t-1) + w(t) * dt
	//	v^(t) = v(t-1) + a(t) * dt
	//	w^(t) = w(t-1)
	//	a^(t) = a(t-1)
	X_estimate(4) = X_previous(4); // w_t
	X_estimate(5) = X_previous(5); // a_t
	X_estimate(3) = X_previous(3) + X_estimate(5) * dt; // v_t
	X_estimate(2) = X_previous(2) + X_estimate(4) * dt; // theta_t
	X_estimate(0) = X_previous(0) + X_estimate(3) * ( ::cos( X_estimate(2) ) * dt ); // x_t
	X_estimate(1) = X_previous(1) + X_estimate(3) * ( ::sin( X_estimate(2) ) * dt ); // y_t

	// ---------- 2. prediction of the observation -------------//
	//	Z^(t) = H * X^;
	Z_estimate = H * X_estimate;

	// ---------- 3. cacluate the Jacobian Matrix of the state function ------------//
	//	F = | 1, 0, -v(t)*sin(theta(t))*dt, cos(theta(t))*dt, 0,  0  |
	//	    | 0, 1,  v(t)*cos(theta(t))*dt, sin(theta(t))*dt, 0,  0  |
	//	    | 0, 0, 	    1,			 0,    	      dt, 0  |
	//	    | 0, 0, 	    0, 			 1, 	      0,  dt |
	//	    | 0, 0, 	    0, 			 0, 	      1,  0  |	
	// 	    | 0, 0, 	    0, 			 0,	      0,  0  |
	F(0, 2) = -X_estimate(3) * ( ::sin( X_estimate(2) ) ) * dt;
	F(0, 3) = ( ::cos( X_estimate(2) ) ) * dt;
	F(1, 2) = X_estimate(3) * ( ::cos( X_estimate(2) ) ) *dt;
	F(1, 3) = ( ::sin( X_estimate(2) ) ) * dt;
	F(2, 4) = dt;
	F(3, 5) = dt;
	
	// ---------- 4. prediction of the covariance matrix ------------//
	//	P^(t) = F * P(t-1) * F.transpose() + Q
	P_estimate = F * P_previous * F.transpose() + Q;
}

void EKF::update( const float vr, const float vl, const float w, const float a )
{
	// ----------- 5. caculate Kalman Gain ------------//
	Eigen::Matrix<float, 4, 4> temp;
	temp = H * P_estimate.inverse() * H.transpose() + R;
	Eigen::Matrix<float, 6, 4> K;
	K = P_estimate * H.transpose() * temp.inverse(); 

	// ----------- 6. input the observation ------------//
	setInputObservation( vr, vl, w, a );

	// ----------- 7. state update ---------------------//
	X = X_estimate + K * ( Z - Z_estimate );

	// ----------- 8. covariance matrix update --------//
	Eigen::Matrix<float, 6, 6> I; 
	I = Eigen::Matrix<float, 6, 6>::Identity();
	P = ( I - K * H ) * P_estimate;

	X_previous = X;
	P_previous = P;
}

void EKF::setInputObservation( const float vr, const float vl, const float w, const float a )
{	
	Z << vr, 	
	     vl,
	     w,
	     a;	
}

const Eigen::Matrix<float, 6, 1> EKF::getStateX()
{
	return X;
}

void EKF::setL( const float l )
{
	this->l = l;
}

void EKF::setDeltaT( const float dt )
{
	this->dt = dt;
}

void EKF::setNoiseR( const Eigen::Matrix<float, 4, 4> &R )
{
	this->R = R;
}
	
void EKF::setNoiseR( const float r1, const float r2, const float r3, const float r4 )
{
	R(0, 0) = r1;
	R(1, 1) = r2;
	R(2, 2) = r3;
	R(3, 3) = r4;
}
	
void EKF::setNoiseQ( const Eigen::Matrix<float, 6, 6> &Q )
{
	this->Q = Q;
}
	
void EKF::setNoiseQ( const float q1, const float q2, const float q3, 
		      	const float q4, const float q5, const float q6 )
{
	this->Q(0, 0) = q1;
	this->Q(1, 1) = q2;
	this->Q(2, 2) = q3;
	this->Q(3, 3) = q4;
	this->Q(4, 4) = q5;
	this->Q(5, 5) = q6;
}
	
}


}
