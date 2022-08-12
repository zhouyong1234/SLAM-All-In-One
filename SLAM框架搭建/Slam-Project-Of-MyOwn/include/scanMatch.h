#ifndef __SCAN_MATCH_H_
#define __SCAN_MATCH_H_

#include <cmath>
#include <Eigen/Dense>
#include "occupiedMap.h"

namespace slam {

class ScanMatchMethod
{
public:
	ScanMatchMethod();
	~ScanMatchMethod();

	float bilinearInterpolation( const OccupiedMap &occuMap, const Eigen::Vector2f &coords );	

	Eigen::Vector3f bilinearInterpolationWithDerivative( const OccupiedMap &occuMap, const Eigen::Vector2f &coords );
	

	void getHessianDerivative( const OccupiedMap &occuMap,
				   Eigen::Vector3f &robotPoseInWorld,
				   const ScanContainer &scanPoints,
				   Eigen::Matrix3f &H, 
				   Eigen::Vector3f &dTr );

	bool estimateTransformationOnce( const OccupiedMap &occuMap, 
					 Eigen::Vector3f &estimateInWorld, 
					 const ScanContainer &scanPoints );

	void updateEstimatedPose( const OccupiedMap &occuMap, Eigen::Vector3f &estimateInWorld, Eigen::Vector3f &delta );

	Eigen::Vector3f scanToMap( const OccupiedMap &occuMap,
			Eigen::Vector3f &beginEstimatedPoseInWorld,
			const ScanContainer &scanPoints,
			Eigen::Matrix3f &covarinceMatrix,
			int maxInterations );	

private:
	Eigen::Matrix3f H;
	Eigen::Vector3f dTr;	

	float mP00;
	float mP11;
	float mP01;
	float mP10;
};



}


#endif
