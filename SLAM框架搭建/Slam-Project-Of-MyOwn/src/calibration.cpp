#include "calibration.h"
#include <iostream>

namespace slam {

namespace sensor{

Calibration::Calibration() : maxIteration(100), accuracy(0.000000001), step(100.0)
{

}

Calibration::~Calibration()
{

}

void Calibration::addDataOfAccelerometerXUpwards( const IMU &imu )
{
	accelArrayXUpwards.clear();

	Eigen::Vector3f accel( 0.0f, 0.0f, 0.0f );

        accel << imu.ax,
                 imu.ay,
                 imu.az;

        accelArrayXUpwards.push_back( accel );
}

void Calibration::addDataOfAccelerometerXDownwards( const IMU &imu )
{
        accelArrayXDownwards.clear();

        Eigen::Vector3f accel( 0.0f, 0.0f, 0.0f );

        accel << imu.ax,
                 imu.ay,
                 imu.az;

        accelArrayXDownwards.push_back( accel );
}

void Calibration::addDataOfAccelerometerYUpwards( const IMU &imu )
{
        accelArrayYUpwards.clear();

        Eigen::Vector3f accel( 0.0f, 0.0f, 0.0f );

        accel << imu.ax,
                 imu.ay,
                 imu.az;

        accelArrayYUpwards.push_back( accel );
}

void Calibration::addDataOfAccelerometerYDownwards( const IMU &imu )
{
        accelArrayYDownwards.clear();

        Eigen::Vector3f accel( 0.0f, 0.0f, 0.0f );

        accel << imu.ax,
                 imu.ay,
                 imu.az;

        accelArrayYDownwards.push_back( accel );
}

void Calibration::addDataOfAccelerometerZUpwards( const IMU &imu )
{
        accelArrayZUpwards.clear();

        Eigen::Vector3f accel( 0.0f, 0.0f, 0.0f );

        accel << imu.ax,
                 imu.ay,
                 imu.az;

        accelArrayZUpwards.push_back( accel );
}

void Calibration::addDataOfAccelerometerZDownwards( const IMU &imu )
{
        accelArrayZDownwards.clear();

        Eigen::Vector3f accel( 0.0f, 0.0f, 0.0f );

        accel << imu.ax,
                 imu.ay,
                 imu.az;

        accelArrayZDownwards.push_back( accel );
}


bool Calibration::caculInputData()
{
	if( accelArrayXUpwards.empty() || 
		accelArrayXDownwards.empty() || 
		accelArrayYUpwards.empty()  || 
		accelArrayYDownwards.empty() || 
		accelArrayZUpwards.empty() || 
		accelArrayZDownwards.empty() ){
		return false;
	}
	
	for( int i = 0; i < 6; i ++ ){
		inputData[i] = Eigen::Vector3f::Zero();
	}	

	// Caculate inputData[0]
	for( auto it : accelArrayXUpwards ){
		inputData[0](0) += it[0];
		inputData[0](1) += it[1];
		inputData[0](2) += it[2];
	}
	inputData[0](0) /= accelArrayXUpwards.size();
	inputData[0](1) /= accelArrayXUpwards.size();
	inputData[0](2) /= accelArrayXUpwards.size();
	
	// Caculate inputData[1]
	for( auto it : accelArrayXDownwards ){
                inputData[1](0) += it[0];
                inputData[1](1) += it[1];
                inputData[1](2) += it[2];
        }
        inputData[1](0) /= accelArrayXDownwards.size();
        inputData[1](1) /= accelArrayXDownwards.size();
        inputData[1](2) /= accelArrayXDownwards.size();

	// Caculate inputData[2]
	for( auto it : accelArrayYUpwards ){
                inputData[2](0) += it[0];
                inputData[2](1) += it[1];
                inputData[2](2) += it[2];
        }
        inputData[2](0) /= accelArrayYUpwards.size();
        inputData[2](1) /= accelArrayYUpwards.size();
        inputData[2](2) /= accelArrayYUpwards.size();

	// Caculate inputData[3]
	for( auto it : accelArrayYDownwards ){
                inputData[3](0) += it[0];
                inputData[3](1) += it[1];
                inputData[3](2) += it[2];
        }
        inputData[3](0) /= accelArrayYDownwards.size();
        inputData[3](1) /= accelArrayYDownwards.size();
        inputData[3](2) /= accelArrayYDownwards.size();

	// Caculate inputData[4]
	for( auto it : accelArrayZUpwards ){
                inputData[4](0) += it[0];
                inputData[4](1) += it[1];
                inputData[4](2) += it[2];
        }
        inputData[4](0) /= accelArrayZUpwards.size();
        inputData[4](1) /= accelArrayZUpwards.size();
        inputData[4](2) /= accelArrayZUpwards.size();
	
	// Caculate inputData[5]
	for( auto it : accelArrayZDownwards ){
                inputData[5](0) += it[0];
                inputData[5](1) += it[1];
                inputData[5](2) += it[2];
        }
        inputData[5](0) /= accelArrayZDownwards.size();
        inputData[5](1) /= accelArrayZDownwards.size();
        inputData[5](2) /= accelArrayZDownwards.size();

	// print the results
	for( int i = 0; i < 6; i ++ ){
		std::cout<<"inputData["<<i<<"] = "<<std::endl;
		std::cout<<inputData[i]<<std::endl;
	}
	
	return true;
}

void Calibration::updateHessianJacobian( const Eigen::Vector3f &data, float residual  )
{
	float b = 0.0f, dx = 0.0f;

	for( int i = 0; i < 3; i ++ ){
		b = beta[3 + i];
		dx = data[i] - beta[i];

		residual -= b * b * dx * dx;

		Jacobian[i] = 2.0f * b * b * dx;
		Jacobian[i + 3] = -2.0f * b * dx * dx;
	}

	Hessian = Jacobian.transpose() * Jacobian;
}


void Calibration::setInitialSolutionValue( const float offsetX, const float offsetY, const float offsetZ, const float scaleX, const float scaleY, const float scaleZ )
{
	beta << offsetX,
		offsetY,
		offsetZ, 
		scaleX,
		scaleY, 
		scaleZ;
}

bool Calibration::calibrateAccelerometer()
{
	float residual = 1.0f;
	
	Eigen::Vector3f data( 0.0f, 0.0f, 0.0f );

	Eigen::Matrix<float, 6, 1> delta;
	delta << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
	
	// 1. set the initial value of the solution
	
	// 2. iteration
	int count = 0;
	while( step > accuracy && count < maxIteration ){
		count ++;
		
		// 3. initial the jacobian and hessian 
		Hessian = Eigen::Matrix<float, 6, 6>::Zero();
		Jacobian = Eigen::Matrix<float, 6, 1>::Zero();

		// 4. caculate the hessian and jacobian
		for( int i = 0; i < 6; i ++ ){
			data(0) = inputData[i](0);
			data(1) = inputData[i](1);
			data(2) = inputData[i](2);
			
			updateHessianJacobian( data, residual );
		}

		// 5. solve the equation
		delta = Hessian.inverse() * ( Jacobian.transpose() * residual );	
	
		// 6. Caculate the iteration step size
		step = delta[0] * delta[0] +
			delta[0] * delta[0] +
                 	delta[1] * delta[1] +
                 	delta[2] * delta[2] +
                 	delta[3] * delta[3] / ( beta[3] * beta[3] ) +
                 	delta[4] * delta[4] / ( beta[4] * beta[4] ) +
                 	delta[5] * delta[5] / ( beta[5] * beta[5] );
	
		// 7. update the equation solution
		beta -= delta;		
	}
	
	return true;
}

}

}
