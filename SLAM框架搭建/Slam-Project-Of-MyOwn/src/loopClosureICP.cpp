#include "loopClosureICP.h"

namespace slam{

ICPLoopClosure::ICPLoopClosure()
{

}

ICPLoopClosure::~ICPLoopClosure()
{

}

void ICPLoopClosure::detectLoop( const slam::sensor::LaserScan &scan )
{
	scanVec.push_back( scan );
	
	//int loop_id = -1;
	matchedScanID = -1;	

	Eigen::Vector3f currPose = poseVec.back();
	
	if( poseVec.size() < NUM_EXCLUDE_RECENT + 1 ){
		return ;	
	}

	// 1. reconstruct the kd tree
	if( tree_making_period_counter % TREE_MAKING_PERIOD == 0 ){
		poseMat.clear();
		poseMat.assign( poseVec.begin(), poseVec.end() - NUM_EXCLUDE_RECENT );
		kdTree.reset();
		
		kdTree = std::make_unique<myKDTree>( 3, poseMat, 10 );
	
		std::cout<<"................. Reconstruct the kd tree ..................... "<<std::endl;
	}

	tree_making_period_counter += 1;
	
	// 2. knn search
	std::vector<size_t> candidate_indexes( NUM_CANDIDATES_FROM_TREE );
        std::vector<float> out_dists_sqr( NUM_CANDIDATES_FROM_TREE );
	
	nanoflann::KNNResultSet<float> knnsearch_result( NUM_CANDIDATES_FROM_TREE );
	knnsearch_result.init( &candidate_indexes[0], &out_dists_sqr[0] );
	
        kdTree->index->findNeighbors( knnsearch_result, &currPose[0], nanoflann::SearchParams(10) );

	float minLoss = 100000.0f, loss = 0.0f;

	for ( int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++ ){
		sensor::LaserScan scanCandidate = scanVec[ candidate_indexes[ candidate_iter_idx ] ];
	
		ScanContainer pointsCandidate;
		ScanContainer pointsNow;
	
		pointsNow.pointTransform2LaserCoords( scan );
		pointsCandidate.pointTransform2LaserCoords( scanCandidate );
	
		loss = icp.solveICP( pointsCandidate, pointsNow );

		if( loss < minLoss ){
			minLoss = loss;
			matchedScanID = candidate_indexes[ candidate_iter_idx ];
		}
	
	}

}

void ICPLoopClosure::caculateTransformByICP()
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


const Eigen::Matrix<float, 2, 2> ICPLoopClosure::getRotateMatrix() const
{
	return icp.getRotateMatrix();
}

const Eigen::Vector2f ICPLoopClosure::getTransformVector() const
{
	return icp.getTransform();
}


void ICPLoopClosure::setPose( const Eigen::Vector3f &pose )
{
	poseVec.push_back( pose );
}

int ICPLoopClosure::detectedALoop() const 
{
	return ( matchedScanID );
}


}
