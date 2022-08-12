#ifndef __POINT_CLOUD_MAP_H_
#define __POINT_CLOUD_MAP_H_

#include <vector>
#include <iostream>

#include <Eigen/Dense>

namespace slam{

template<typename BaseType, int Dimension>
class PointCloudMap
{
public:
	typedef Eigen::Matrix<BaseType, Dimension, 1> PointType;
public:
	PointCloudMap();
	~PointCloudMap();

	const PointType pointPosistionLaser2World( const PointType &pointInLaser );
	const PointType pointPositionWorld2Laser( const PointType &pointInWorld );

private:
	std::vector<Eigen::Matrix<BaseType, Dimension, 1>> pointCloud;

};

}

#endif
