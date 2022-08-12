#ifndef __IMU_CALIBRATION_H_
#define __IMU_CALIBRATION_H_

#include <Eigen/Dense>
#include <vector>

#include "dataType.h"

namespace slam{

namespace sensor{

class IMUCalibration
{
public:
	IMUCalibration();
	~IMUCalibration();

	virtual void addDataOfAccelerometer( const IMU &imu );
	void addDataOfGyrometer( const IMU &imu );

	virtual bool calibrateAccelerometer();
	bool calibrateGyrometer();

	const Eigen::RowVector3f getAccelBias() const;
	const Eigen::Vector3f getGyroBias() const;

	const float getAccelBiasX() const;
	const float getAccelBiasY() const;
	const float getAccelBiasZ() const;

	const float getGyroBiasX() const;
	const float getGyroBiasY() const;
	const float getGyroBiasZ() const;

private:
	template<typename T>
	T square(const T &num );

	bool caculateParametersA();

private:
	std::vector<Eigen::Matrix<float, 9, 1>> accelArray;
	std::vector<Eigen::Vector3f> gyroArray;

	Eigen::Matrix<float, 9, 1> parameters;
	Eigen::RowVector3f ellipsoidCenter;

	Eigen::Vector3f gyroBias;
};

}

}

#endif
