#ifndef __LOOP_CLOSURE_ICP_H_
#define __LOOP_CLOSURE_ICP_H_

#include "loopClosureBase.h"
#include "KDTreeVectorOfVectorsAdaptor.h"
#include <vector>
#include "scanContainer.h"
#include <memory>

#include "icp.h"

namespace slam{

class ICPLoopClosure
{
	using myVectors = std::vector<Eigen::Vector3f>;
	using myKDTree = KDTreeVectorOfVectorsAdaptor<myVectors, float>;

public:
	ICPLoopClosure();
	virtual ~ICPLoopClosure();

	virtual void detectLoop( const slam::sensor::LaserScan &scan );
        virtual void caculateTransformByICP();

        virtual const Eigen::Matrix<float, 2, 2> getRotateMatrix() const;
        virtual const Eigen::Vector2f getTransformVector() const;

	virtual void setPose( const Eigen::Vector3f &pose );

	virtual int detectedALoop() const;
	
private:
	ICP icp;
	
	int matchedScanID = -1;

	myVectors poseMat;
	
	std::vector<Eigen::Vector3f> poseVec;
	std::vector<slam::sensor::LaserScan> scanVec;	

	std::unique_ptr<myKDTree> kdTree;

	const int NUM_EXCLUDE_RECENT = 100;
		
	const int TREE_MAKING_PERIOD = 50;
	int tree_making_period_counter = 0;

	const int NUM_CANDIDATES_FROM_TREE = 10; // 10 is enough. (refer the IROS 18 paper)
};

}

#endif
