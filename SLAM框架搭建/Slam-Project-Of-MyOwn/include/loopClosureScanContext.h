#ifndef __LOOP_CLOSURE_SCAN_CONTEXT_H_
#define __LOOP_CLOSURE_SCAN_CONTEXT_H_

#include "loopClosureBase.h"
#include "scanContext.h"

namespace slam{

class ScanContextLoopClosure : public LoopClosureBase
{
public:
	ScanContextLoopClosure();
	virtual ~ScanContextLoopClosure();

	virtual void detectLoop( const slam::sensor::LaserScan &scan );
	virtual void caculateTransformByICP();

	virtual const Eigen::Matrix<float, 2, 2> getRotateMatrix() const;
	virtual const Eigen::Vector2f getTransformVector() const;

	virtual void setPose( const Eigen::Vector3f &pose );

	virtual int detectedALoop() const;

	virtual bool saveScanContext(const std::string &filename, const std::vector<Eigen::Vector3f> &keyPoses);

private:
	ICP icp;
	ScanContext<float, 20> scanContext;
	std::vector<slam::sensor::LaserScan> scanVec;
	
	int matchedScanID = -1;
	float angleBias = 0.0f;
	
};



}



#endif


