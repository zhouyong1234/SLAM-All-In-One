#ifndef __LOOP_CLOSURE_BASE_H_
#define __LOOP_CLOSURE_BASE_H_

#include "dataType.h"
#include "scanContainer.h"
#include "icp.h"

namespace slam{

class LoopClosureBase{
public:
	LoopClosureBase()
	{
	
	}
	
	virtual ~LoopClosureBase()
	{

	}

	virtual void detectLoop( const slam::sensor::LaserScan &scan ) = 0;
	virtual void caculateTransformByICP() = 0;
	
	virtual const Eigen::Matrix<float, 2, 2> getRotateMatrix() const  = 0;
	virtual const Eigen::Vector2f getTransformVector() const = 0;
	
	virtual void setPose( const Eigen::Vector3f &pose ) = 0;

	virtual int detectedALoop() const = 0;

	virtual bool saveScanContext(const std::string &filename, const std::vector<Eigen::Vector3f> &keyPoses) = 0;
};

}

#endif
