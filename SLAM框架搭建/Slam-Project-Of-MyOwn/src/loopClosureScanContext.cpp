#include "loopClosureScanContext.h"
#include <fstream>

namespace slam{

ScanContextLoopClosure::ScanContextLoopClosure()
{

}

ScanContextLoopClosure::~ScanContextLoopClosure()
{

}

void ScanContextLoopClosure::detectLoop( const slam::sensor::LaserScan &scan )
{
	scanVec.push_back( scan );

	scanContext.makeAndSaveScancontextAndKeys( scan );

	std::pair<int, float> ret = scanContext.detectLoopClosureID();

	matchedScanID = ret.first;
	angleBias = ret.second;
	// std::cout << "matchedScanID: " << matchedScanID << " angleBias: " << angleBias << std::endl;
}

bool ScanContextLoopClosure::saveScanContext(const std::string &filename, const std::vector<Eigen::Vector3f> &keyPoses) 
{
	std::fstream stream;
        stream.open(filename, std::ios::app);

	std::vector<Eigen::MatrixXf> ScanContexts = scanContext.getScanContexts();
	std::vector<Eigen::MatrixXf> RingKeys = scanContext.getRingKeys();
	std::vector<Eigen::MatrixXf> SectorKeys = scanContext.getSectorKeys();

	for( int i = 1; i < keyPoses.size(); i++ ){
              stream << "keyPoses " << keyPoses.size() - 1 <<" "<< keyPoses[i][0] << " " << keyPoses[i][1] << " " << keyPoses[i][2] << std::endl;
        }


	for( int i = 0; i < RingKeys.size(); i ++ ){
		stream << "RingKeys " << RingKeys[i].size() << " ";
		for( int j = 0; j < RingKeys[i].rows(); j ++ ){
			stream << RingKeys[i](j, 0) << " ";
		}
		stream << std::endl;
	}
	for( int i = 0; i < SectorKeys.size(); i ++ ){
		stream << "SectorKeys " << SectorKeys[i].size() << " ";
		stream << SectorKeys[i] << " ";
		stream << std::endl;
	}
	for (int i = 0; i < ScanContexts.size(); i++){
		stream << "ScanContexts " << ScanContexts[i].size() << " ";
		for (int j = 0; j < ScanContexts[i].rows(); j++){
			for (int k = 0; k < ScanContexts[i].cols(); k++){
				stream << ScanContexts[i](j, k) << " ";
			}
		}
		stream << std::endl;
	}

	stream.close();
}

void ScanContextLoopClosure::caculateTransformByICP()
{
	if( matchedScanID != -1 ){
		ScanContainer pointsCandidate;
		ScanContainer pointsNow;

		sensor::LaserScan scanNow = scanVec.back();
		sensor::LaserScan scanCandidate = scanVec[ matchedScanID ];

		pointsCandidate.pointTransform2LaserCoords( scanCandidate ) ;
		pointsNow.pointTransform2LaserCoords( scanNow );

		float loss = icp.solveICP( pointsCandidate, pointsNow );
		std::cout<<"loss = "<<loss<<std::endl;
	}
}

const Eigen::Matrix<float,2 ,2> ScanContextLoopClosure::getRotateMatrix() const 
{
	return icp.getRotateMatrix();
}

const Eigen::Vector2f ScanContextLoopClosure::getTransformVector() const
{
	return icp.getTransform();
}

void ScanContextLoopClosure::setPose( const Eigen::Vector3f &pose )
{

}

int ScanContextLoopClosure::detectedALoop() const 
{
	return ( matchedScanID );
}



}
