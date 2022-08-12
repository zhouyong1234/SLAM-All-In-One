#include <iostream>
#include "IMUCalibration.h"

namespace slam{

namespace sensor{

IMUCalibration::IMUCalibration()
{

}

IMUCalibration::~IMUCalibration()
{

}

template<typename T>
T IMUCalibration::square(const T &num)
{
	return num * num;
}

void IMUCalibration::addDataOfAccelerometer(const IMU &imu)
{
	Eigen::Matrix<float, 9, 1> accel;

	accel << square<float>(imu.ax),
			 square<float>(imu.ay),
			 square<float>(imu.az),
			 imu.ax * imu.ay,
			 imu.ax * imu.az,
			 imu.ay * imu.az,
			 imu.ax,
			 imu.ay,
			 imu.az;

	accelArray.push_back( accel );
}

void IMUCalibration::addDataOfGyrometer(const IMU &imu)
{
	Eigen::Vector3f gyro;

	gyro << imu.gx,
			imu.gy,
			imu.gz;

	gyroArray.push_back( gyro );
}

bool IMUCalibration::caculateParametersA()
{
	if (accelArray.empty()) {
		return false;
	}

	int rows = accelArray.size();
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> H(rows, 9);

	Eigen::VectorXf Y(9);
	Y << 1, 
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1;
	std::cout << "Y = " << std::endl << Y << std::endl;

	for (int i = 0; i < rows; i ++) {
		H.row(i) = accelArray[i];
	}

	Eigen::Matrix<float, 9, 9> H_tmp;
	H_tmp = H.transpose() * H;

	parameters = H_tmp.inverse() * H.transpose() * Y;
	std::cout << "parameters = " << std::endl << parameters << std::endl;
	
	return true;
}

bool IMUCalibration::calibrateAccelerometer()
{
	Eigen::Matrix<float, 3, 3> M = Eigen::Matrix<float, 3, 3>::Zero();

	M(0, 0) = parameters[0];
	M(1, 1) = parameters[1];
	M(2, 2) = parameters[2];
	
	M(0, 1) = parameters[3] * 0.5f;
	M(0, 2) = parameters[4] * 0.5f;
	M(1, 2) = parameters[5] * 0.5f;

	M(1, 0) = M(0, 1);
	M(2, 0) = M(0, 2);
	M(2, 1) = M(1, 2);

	std::cout << "M = " << std::endl << M << std::endl;

	Eigen::RowVector3f tmp( parameters[6], parameters[7], parameters[8]);

	ellipsoidCenter = -0.5f * tmp * M.inverse();

	return true;
}

bool IMUCalibration::calibrateGyrometer()
{
	if (gyroArray.empty()) {
		return false;
	}

	float ax_sum = 0, ay_sum = 0, az_sum = 0;

	for (auto it : gyroArray) {
		ax_sum += it[0];
		ay_sum += it[1];
		az_sum += it[2];
	}
	std::cout << "ax sum = " << ax_sum << std::endl;
	std::cout << "ay sum = " << ay_sum << std::endl;
	std::cout << "az sum = " << az_sum << std::endl;

	float count = static_cast<float>( gyroArray.size() );
	gyroBias = Eigen::Vector3f(ax_sum / count, ay_sum / count, az_sum / count );

	return true;
}

const Eigen::RowVector3f IMUCalibration::getAccelBias() const
{
	return ellipsoidCenter;
}

const Eigen::Vector3f IMUCalibration::getGyroBias() const
{
	return gyroBias;
}

const float IMUCalibration::getAccelBiasX() const
{
	return ellipsoidCenter[0];
}

const float IMUCalibration::getAccelBiasY() const
{
	return ellipsoidCenter[1];
}

const float IMUCalibration::getAccelBiasZ() const
{
	return ellipsoidCenter[2];
}

const float IMUCalibration::getGyroBiasX() const
{
	return gyroBias[0];
}

const float IMUCalibration::getGyroBiasY() const
{
	return gyroBias[1];
}

const float IMUCalibration::getGyroBiasZ() const
{
	return gyroBias[2];
}

}

}
