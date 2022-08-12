#ifndef __ICP_H_
#define __ICP_H_

#include <Eigen/Dense>
#include "scanContainer.h"
#include <vector>

namespace slam{

class ICP{
public:
	ICP();
	~ICP();

	const float solveICP( ScanContainer& A_src, ScanContainer& B_src );
	
	inline const Eigen::Matrix<float, 2, 2> getRotateMatrix() const
	{
		return R_final;
	}
	
	inline const Eigen::Vector2f getTransform() const 
	{
		return T_final;
	}

	
private:
	const float iterateOnce( std::vector<Eigen::Vector2f>& B );

	const Eigen::Vector2f getClosestPoint( const Eigen::Vector2f &point, const std::vector<Eigen::Vector2f> &sourcePoints );

	const int getClosestPointID( const Eigen::Vector2f &point, const std::vector<Eigen::Vector2f> &sourcePoints );

	const float cacuLoss( const std::vector<Eigen::Vector2f> &B ); 
	
private:
	Eigen::Vector2f Acenter;
	Eigen::Vector2f Bcenter;

	std::vector<Eigen::Vector2f> A;
//        std::vector<Eigen::Vector2f> B;	

	std::vector<Eigen::Vector2f> a;
	std::vector<Eigen::Vector2f> b;	

	Eigen::Matrix<float, 2, 2> R;
	Eigen::Vector2f T;
	
	Eigen::Matrix<float, 2, 2> R_final = Eigen::Matrix<float, 2, 2>::Identity();
	Eigen::Vector2f T_final = Eigen::Vector2f( 0.0f, 0.0f ) ;

	int maxIteration = 100;
};

}


#endif
