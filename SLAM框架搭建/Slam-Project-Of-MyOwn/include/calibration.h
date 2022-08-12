#ifndef __CALIBRATION_H_
#define __CALIBRATION_H_

#include "IMUCalibration.h"

namespace slam{

namespace sensor{

class Calibration final : public IMUCalibration
{
public:
	Calibration();
	~Calibration();

	void addDataOfAccelerometerXUpwards( const IMU &imu );
	void addDataOfAccelerometerXDownwards( const IMU &imu );
	void addDataOfAccelerometerYUpwards( const IMU &imu );
        void addDataOfAccelerometerYDownwards( const IMU &imu );
	void addDataOfAccelerometerZUpwards( const IMU &imu );
        void addDataOfAccelerometerZDownwards( const IMU &imu );

	void addDataOfAccelerometerY();
	
	virtual bool calibrateAccelerometer();

	void setInitialSolutionValue( const float offsetX, const float offsetY, const float offsetZ, const float scaleX, const float scaleY, const float scaleZ );

	inline const Eigen::Matrix<float, 6, 1> getBeta() const 
	{
		return beta;
	}
	
	inline const float getOffsetX() const
	{
		return beta(0);	
	}
	
	inline const float getOffsetY() const
        {       
                return beta(1);
        }

	inline const float getOffsetZ() const
        {       
                return beta(2);
        }

	inline const float getScaleX() const 
	{
		return beta(3);
	}
	
	inline const float getScaleY() const
        {
                return beta(4);
        }

	inline const float getScaleZ() const
        {
                return beta(5);
        }

	inline void setMaxIteration( int maxIteration )
	{
		this->maxIteration = maxIteration;
	}

	inline void setAccuracy( double accuracy )
	{
		this->accuracy = accuracy;
	}

	inline void setStepValue( double step )
	{
		this->step = step;
	}	

private:
	bool caculInputData();
	void updateHessianJacobian( const Eigen::Vector3f &data, float residual );
	
private:
	std::vector<Eigen::Vector3f> accelArrayXUpwards;
	std::vector<Eigen::Vector3f> accelArrayXDownwards;
	std::vector<Eigen::Vector3f> accelArrayYUpwards;
        std::vector<Eigen::Vector3f> accelArrayYDownwards;
	std::vector<Eigen::Vector3f> accelArrayZUpwards;
        std::vector<Eigen::Vector3f> accelArrayZDownwards;
	
	Eigen::Vector3f inputData[6];

	int maxIteration;
	double accuracy;
	double step;
	
	Eigen::Matrix<float, 6, 6> Hessian;
	Eigen::Matrix<float, 1, 6> Jacobian;
	
	Eigen::Matrix<float, 6, 1> beta;
};

}

}


#endif
