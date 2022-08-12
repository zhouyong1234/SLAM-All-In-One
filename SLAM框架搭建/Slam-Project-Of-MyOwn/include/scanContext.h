#ifndef __SCAN_CONTEXT_H_
#define __SCAN_CONTEXT_H_

#include "KDTreeVectorOfVectorsAdaptor.h"
#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>
#include <utility>

#include "dataType.h"

#include <memory>

#include <opencv2/opencv.hpp>

#include <map>

namespace slam{

struct yaw_dist_id
{
	float yaw;
	float dist;
	int pose_id;
};

// just for visulizing the scan context, not important
static const unsigned char r[64] = { 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   10,  20,  30,  40,  50,  60,  70,  80,  90,  100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230, 240, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255 };
static const unsigned char g[64] = { 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 240, 230, 220, 210, 200, 190, 180, 170, 160, 150, 140, 130, 120, 110, 100, 90,  80,  70,  60,  50,  40,  30,  20,  10,  0 };
static const unsigned char b[64] = { 255, 240, 220, 200, 180, 160, 140, 120, 100, 80,  60,  40,  20,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0 };


template<typename T, int Dimension = 3>
class ScanContext
{

	using myVectors = std::vector<Eigen::Matrix<T, Dimension, 1>>;
	using myKDTree = KDTreeVectorOfVectorsAdaptor<myVectors, T>;
	using MatrixType = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

public:
	ScanContext();
	ScanContext( int NUM_RING_, int NUM_SECTOR_, float MAX_RADIUS_, 
		     float SEARCH_RATIO_, int NUM_EXCLUDE_RECENT_, 
		     int TREE_MAKING_PERIOD_, int NUM_CANDIDATES_FROM_TREE_, 
		     double SC_DIST_THRES_ );

	~ScanContext();

	const Eigen::MatrixXf makeScanContext( const slam::sensor::LaserScan &scan );
	const Eigen::MatrixXf makeRingkeyFromScancontext( const Eigen::MatrixXf &desc );
    	const Eigen::MatrixXf makeSectorkeyFromScancontext( const Eigen::MatrixXf &desc );
	void makeAndSaveScancontextAndKeys( const slam::sensor::LaserScan &scan );
	
	const std::pair<int, float> detectLoopClosureID();

	void detectScancontextID( yaw_dist_id &yaw_dist_id_);

	void displayAScancontext( const Eigen::MatrixXf &desc ) const;
private:
	template<typename TT>
	const TT rad2deg( const TT radians)
	{
    		return radians * 180.0 / M_PI;
	}

	template<typename TT>
	const TT deg2rad( const TT angle )
	{
		return angle * M_PI / 180.0;
	}

	const Eigen::MatrixXf circshift( const Eigen::MatrixXf &_mat, const int _num_shift );
	const float cosineDistance( const Eigen::MatrixXf &_sc1, const Eigen::MatrixXf &_sc2 );
	const int fastAlignUsingVkey( const Eigen::MatrixXf &_vkey1, const Eigen::MatrixXf &_vkey2);

	const std::pair<float, int> distanceBetweenScancontexts( const Eigen::MatrixXf &_sc1, const Eigen::MatrixXf &_sc2 );
	
	// added
	void drawABin(cv::Mat &image,  int ring_idx, const int sctor_idx);
	
public:
	
	// added 
	const Eigen::MatrixXf& getScanContext( int index ) const;
	const Eigen::MatrixXf& getRingKey( int index ) const; 
	const int getScanContextsSize() const;

	const std::vector<Eigen::MatrixXf> getScanContexts() const
	{
		return scanContexts;
	}

	const std::vector<Eigen::MatrixXf> getRingKeys() const
	{
		return ringKeys;
	}	

	const std::vector<Eigen::MatrixXf> getSectorKeys() const
	{
		return sectorKeys;
	}

	void setScanContexts(const std::vector<Eigen::MatrixXf> scanContexts_)
	{
		scanContexts = scanContexts_;
	}

	void setRingKeys( const std::vector<Eigen::MatrixXf> &ringKeys_)
	{
		ringKeys = ringKeys_;
	}
	void setSectorKeys( const std::vector<Eigen::MatrixXf> &sectorKeys_)
	{
		sectorKeys = sectorKeys_;
	}

	void displayScanDistribution( const slam::sensor::LaserScan &scan );
	void setParameters( int NUM_RING_, int NUM_SECTOR_, float MAX_RADIUS_,
                     	    float SEARCH_RATIO_, int NUM_EXCLUDE_RECENT_,
     			    int TREE_MAKING_PERIOD_, int NUM_CANDIDATES_FROM_TREE_,
        		    double SC_DIST_THRES_  );

private:
	myVectors ringKeysMat;
	std::vector<Eigen::MatrixXf> ringKeys;
	std::vector<Eigen::MatrixXf> sectorKeys;
	std::vector<Eigen::MatrixXf> scanContexts;
	std::unique_ptr<myKDTree> kdTree;

	/*const int NUM_RING = 20;
	const int NUM_SECTOR = 60;
	
	const float MAX_RADIUS = 15.0f;
	const float UNIT_SECTOR_ANGLE = 360.0 / float(NUM_SECTOR);
	const float UNIT_RING_GAP = MAX_RADIUS / float(NUM_RING);
	
	const float SEARCH_RATIO = 0.1; 
	
	const int NUM_EXCLUDE_RECENT = 50;
		
	const int TREE_MAKING_PERIOD = 20;
	int tree_making_period_conter = 0;

	const int    NUM_CANDIDATES_FROM_TREE = 10; // 10 is enough. (refer the IROS 18 paper)

	const double SC_DIST_THRES = 0.17; // empirically 0.1-0.2 is fine (rare false-alarms) for 20x60 polar context (but for 0.15 <, DCS or ICP fit score check (e.g., in LeGO-LOAM) should be required for robustness)
	*/
	
	// ---------------------- Scan Context Parameters ------------------- //
	// need to be changed according to your actual applications
	int NUM_RING = 20;
        int NUM_SECTOR = 60;

        float MAX_RADIUS = 15.0f;
        float UNIT_SECTOR_ANGLE = 360.0 / float(NUM_SECTOR);
        float UNIT_RING_GAP = MAX_RADIUS / float(NUM_RING);

        float SEARCH_RATIO = 0.1;

        int NUM_EXCLUDE_RECENT = 50;

        int TREE_MAKING_PERIOD = 20;
        int tree_making_period_conter = 0;

        int    NUM_CANDIDATES_FROM_TREE = 10; // 10 is enough. (refer the IROS 18 paper)

        double SC_DIST_THRES = 0.17; // empirically 0.1-0.2 is fine (rare false-alarms) for 20x60 polar context (but for 0.15 <, DCS or ICP fit score check (e.g., in LeGO-LOAM) should be required for robustness)

	// ----------------------------- END -------------------------------//
};	

template<typename T, int Dimension>
ScanContext<T, Dimension>::ScanContext()
{

}

template<typename T, int Dimension>
ScanContext<T, Dimension>::~ScanContext()
{

}

template<typename T, int Dimension>
ScanContext<T, Dimension>::ScanContext( int NUM_RING_, int NUM_SECTOR_, float MAX_RADIUS_,
                     			float SEARCH_RATIO_, int NUM_EXCLUDE_RECENT_,
                     			int TREE_MAKING_PERIOD_, int NUM_CANDIDATES_FROM_TREE_,
                     			double SC_DIST_THRES_ ) : 
					NUM_RING( NUM_RING_ ), NUM_SECTOR( NUM_SECTOR_ ), MAX_RADIUS( MAX_RADIUS_ ),
					SEARCH_RATIO( SEARCH_RATIO_ ), NUM_EXCLUDE_RECENT( NUM_EXCLUDE_RECENT_ ),
					TREE_MAKING_PERIOD( TREE_MAKING_PERIOD_ ), NUM_CANDIDATES_FROM_TREE( NUM_CANDIDATES_FROM_TREE_ ),
					SC_DIST_THRES( SC_DIST_THRES_ )
{
	
}

template<typename T, int Dimension>
void ScanContext<T, Dimension>::setParameters( int NUM_RING_, int NUM_SECTOR_, float MAX_RADIUS_,
					       float SEARCH_RATIO_, int NUM_EXCLUDE_RECENT_,
                            		       int TREE_MAKING_PERIOD_, int NUM_CANDIDATES_FROM_TREE_,
                            		       double SC_DIST_THRES_  )
{
	NUM_RING = NUM_RING_;
	NUM_SECTOR = NUM_SECTOR_;
	MAX_RADIUS = MAX_RADIUS_;
	SEARCH_RATIO = SEARCH_RATIO_;
	NUM_EXCLUDE_RECENT = NUM_EXCLUDE_RECENT_;
	TREE_MAKING_PERIOD = TREE_MAKING_PERIOD_;
	NUM_CANDIDATES_FROM_TREE = NUM_CANDIDATES_FROM_TREE_;
	SC_DIST_THRES = SC_DIST_THRES_;
}

/*template<typename T, int Dimension>
const Eigen::MatrixXf ScanContext<T, Dimension>::makeScanContext( const slam::sensor::LaserScan &scan )
{
	// 1. 
	Eigen::MatrixXf desc = Eigen::MatrixXf::Zero(NUM_RING, NUM_SECTOR);

	int ring_idx = 0, sctor_idx = 0;
	float radians = -3.14159f;
	
	for( int i = 0; i < scan.size(); i ++ ){
		float dist = scan.ranges[i];
		float angle = rad2deg<float>( radians ) + 180.0f;
		
		if( dist >= 0.009999998f && dist <= 15.0000000000f ){
			//std::cout<<"------------------------------------"<<std::endl;
			//std::cout<<"dist = "<<dist<<", angle = "<<angle<<std::endl;
			
			ring_idx = std::max( std::min( NUM_RING - 1, int(ceil( (dist / MAX_RADIUS) * NUM_RING )) ), 0 );
        	
			sctor_idx = std::max( std::min( NUM_SECTOR - 1, int(ceil( (angle / 360.0) * NUM_SECTOR )) ), 0 );
			
			//std::cout<<"ring_idx: "<<ring_idx<<", sctor_idx: "<<sctor_idx<<std::endl;

			desc(ring_idx, sctor_idx) += 0.5f;
		
			//std::cout<<"( ring_idx, sector_idx ): ( " <<ring_idx<<", "<<sctor_idx<<" ) = " <<desc(ring_idx, sctor_idx)<<std::endl;
		}

		radians += 0.0043633231f;
	}

	return desc;
}*/

/*
template<typename T, int Dimension>
const Eigen::MatrixXf ScanContext<T, Dimension>::makeScanContext( const slam::sensor::LaserScan &scan )
{
        // 1. 
        Eigen::MatrixXf desc = Eigen::MatrixXf::Zero(NUM_RING, NUM_SECTOR);
        
	std::map<Eigen::Vector2i, Eigen::Vector2f> avarage;
	std::map<Eigen::Vector2i, int> avarage_num;
	std::map<Eigen::Vector2i, Eigen::Matrix<float, 2, 2>> covarince;

	int ring_idx = 0, sctor_idx = 0;
        float radians = -3.14159f;
        
        for( int i = 0; i < scan.size(); i ++ ){
                float dist = scan.ranges[i];
                float angle = rad2deg<float>( radians ) + 180.0f;
                
                if( dist >= 0.009999998f && dist <= 15.0000000000f ){
                        
                        ring_idx = std::max( std::min( NUM_RING - 1, int(ceil( (dist / MAX_RADIUS) * NUM_RING )) ), 0 );
                
                        sctor_idx = std::max( std::min( NUM_SECTOR - 1, int(ceil( (angle / 360.0) * NUM_SECTOR )) ), 0 );
                        

                       // desc(ring_idx, sctor_idx) += 0.5f;
        		
			Eigen::Vector2f point( radians + M_PI, dist );        
			Eigen::Vector2f key( ring_idx, sctor_idx );		
	
			if( !avarage.count( key ) ){
				avarage.insert( std::make_pair( key, point ) );
			}
			else {
				avarage[key] += point;
			}

			if( !avarage_num.count( key ) ){
				avarage_num.insert( std::make_pair( key, 1 ) );
			}
			else {
				avarage_num[key] += 1;
			}
                }

                radians += 0.0043633231f;
        }

	for( auto it : avarage ){
		if( avarage_num.count( it.first ) ){
			it.second /= avarage_num[it.first];
		}
	}

	for( int i = 0; i < scan.size(); i ++ ){
                float dist = scan.ranges[i];
                float angle = rad2deg<float>( radians ) + 180.0f;

                if( dist >= 0.009999998f && dist <= 15.0000000000f ){

                        ring_idx = std::max( std::min( NUM_RING - 1, int(ceil( (dist / MAX_RADIUS) * NUM_RING )) ), 0 );

                        sctor_idx = std::max( std::min( NUM_SECTOR - 1, int(ceil( (angle / 360.0) * NUM_SECTOR )) ), 0 );

                        Eigen::Vector2f point( radians + M_PI, dist );
                        Eigen::Vector2f key( ring_idx, sctor_idx );
                
			if( !avarage.count( key ) ){
				Eigen::Matrix<float, 2, 2> cov = ( point - avarage[key] ) * ( point - avarage[key] ).transpose();
				covarince.insert( std::make_pair( key, cov ) );
			}
			else{
				covarince[key] += ( point - avarage[key] ) * ( point - avarage[key] ).transpose();
			}
		}

                radians += 0.0043633231f;
        }

	for( auto it : covarince ){
                if( avarage_num.count( it.first ) ){
                        it.second /= avarage_num[it.first];
                }
        }

        return desc;
}*/

template<typename T, int Dimension>
const Eigen::MatrixXf ScanContext<T, Dimension>::makeScanContext( const slam::sensor::LaserScan &scan )
{
        // 1. 
        Eigen::MatrixXf desc = Eigen::MatrixXf::Zero(NUM_RING, NUM_SECTOR);


        int ring_idx = 0, sctor_idx = 0;
        float radians = -3.14159f;
        
        for( int i = 0; i < scan.size(); i ++ ){
                float dist = scan.ranges[i];
                float angle = rad2deg<float>( radians ) + 180.0f;
                
                if( dist >= 0.009999998f && dist <= 15.0000000000f ){
                        
                        ring_idx = std::max( std::min( NUM_RING - 1, int(ceil( (dist / MAX_RADIUS) * NUM_RING )) ), 0 );
                
                        sctor_idx = std::max( std::min( NUM_SECTOR - 1, int(ceil( (angle / 360.0) * NUM_SECTOR )) ), 0 );
        	
                
			desc( ring_idx, sctor_idx ) += 1;
		}

                radians += 0.0043633231f;
        }

	//std::cout<<"desc : "<<std::endl<<desc<<std::endl;
	
	// weighted occupied probability, need to be optimized
	for( int i = 0; i < desc.rows(); i ++ ){
		for( int j = 0; j < desc.cols(); j ++ ){
			float index = desc(i, j);
			if( index <= 1 ){
				desc(i, j) = index * 0.1;
			}
			if( index == 2 ){
				desc(i, j) = index * 1;
			}
			else{
				desc(i, j) = index * 3;
			}
		}
	}	

	//std::cout<<" make a frame of scan context "<<std::endl;
	
        return desc;
}

template<typename T, int Dimension>
const Eigen::MatrixXf ScanContext<T, Dimension>::makeRingkeyFromScancontext( const Eigen::MatrixXf &desc )
{
	Eigen::MatrixXf invariant_key(desc.rows(), 1);
    
	for ( int row_idx = 0; row_idx < desc.rows(); row_idx++ ){
        	Eigen::MatrixXf curr_row = desc.row(row_idx);
        	invariant_key(row_idx, 0) = curr_row.mean();
    	}

#ifdef TERMINAL_LOG
	std::cout<< " ------------ Ring Key Vector ------------- "<<std::endl;
	std::cout<< invariant_key<<std::endl;
#endif	

    	return invariant_key;
}

template<typename T, int Dimension>
const Eigen::MatrixXf ScanContext<T, Dimension>::makeSectorkeyFromScancontext( const Eigen::MatrixXf &desc )
{
	Eigen::MatrixXf variant_key(1, desc.cols());

	for ( int col_idx = 0; col_idx < desc.cols(); col_idx++ ){
        	Eigen::MatrixXf curr_col = desc.col(col_idx);
        	variant_key(0, col_idx) = curr_col.mean();
    	}

    	return variant_key;
}

template<typename T, int Dimension>
void ScanContext<T, Dimension>::makeAndSaveScancontextAndKeys( const slam::sensor::LaserScan &scan )
{
	Eigen::MatrixXf sc = makeScanContext( scan );
	Eigen::MatrixXf ringKey = makeRingkeyFromScancontext( sc );
	Eigen::MatrixXf sectorKey = makeSectorkeyFromScancontext( sc );

	scanContexts.push_back( sc );
	ringKeys.push_back( ringKey );
	sectorKeys.push_back( sectorKey );

}

template<typename T, int Dimension>
const Eigen::MatrixXf ScanContext<T, Dimension>::circshift( const Eigen::MatrixXf &_mat, const int _num_shift )
{
	// shift columns to right direction 
    	assert(_num_shift >= 0);

    	if( _num_shift == 0 ){
        	Eigen::MatrixXf shifted_mat( _mat );
        	return shifted_mat; // Early return 
    	}

    	Eigen::MatrixXf shifted_mat = Eigen::MatrixXf::Zero( _mat.rows(), _mat.cols() );
    	for ( int col_idx = 0; col_idx < _mat.cols(); col_idx++ ){
        	int new_location = (col_idx + _num_shift) % _mat.cols();
        	shifted_mat.col(new_location) = _mat.col(col_idx);
    	}

    	return shifted_mat;
}

template<typename T, int Dimension>
const float ScanContext<T, Dimension>::cosineDistance( const Eigen::MatrixXf &_sc1, const Eigen::MatrixXf &_sc2 )
{
	int num_eff_cols = 0; // i.e., to exclude all-nonzero sector
    	float sum_sector_similarity = 0;
    	
	for ( int col_idx = 0; col_idx < _sc1.cols(); col_idx++ ){
        	Eigen::VectorXf col_sc1 = _sc1.col(col_idx);
        	Eigen::VectorXf col_sc2 = _sc2.col(col_idx);
        
       	 	if( col_sc1.norm() == 0 || col_sc2.norm() == 0 )
            		continue; // don't count this sector pair. 

        	float sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());

        	sum_sector_similarity = sum_sector_similarity + sector_similarity;
        	num_eff_cols = num_eff_cols + 1;
    	}
    
    	float sc_sim = sum_sector_similarity / num_eff_cols;
    	
	return 1.0 - sc_sim;
}

template<typename T, int Dimension>
const int ScanContext<T, Dimension>::fastAlignUsingVkey( const Eigen::MatrixXf &_vkey1, const Eigen::MatrixXf &_vkey2)
{
	int argmin_vkey_shift = 0;
    	float min_veky_diff_norm = 10000000;
    
	for ( int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++ ){
        	Eigen::MatrixXf vkey2_shifted = circshift(_vkey2, shift_idx);

        	Eigen::MatrixXf vkey_diff = _vkey1 - vkey2_shifted;

        	float cur_diff_norm = vkey_diff.norm();

        	if( cur_diff_norm < min_veky_diff_norm ){
            		argmin_vkey_shift = shift_idx;
            		min_veky_diff_norm = cur_diff_norm;
        	}
    	}

    	return argmin_vkey_shift;
}

template<typename T, int Dimension>
const std::pair<float, int> ScanContext<T, Dimension>::distanceBetweenScancontexts( const Eigen::MatrixXf &_sc1, const Eigen::MatrixXf &_sc2 )
{
	// 1. fast align using variant key (not in original IROS18)
    	Eigen::MatrixXf vkey_sc1 = makeSectorkeyFromScancontext( _sc1 );
    	Eigen::MatrixXf vkey_sc2 = makeSectorkeyFromScancontext( _sc2 );
    	
	int argmin_vkey_shift = fastAlignUsingVkey( vkey_sc1, vkey_sc2 );

    	const int SEARCH_RADIUS = round( 0.5 * SEARCH_RATIO * _sc1.cols() ); // a half of search range 
    	std::vector<int> shift_idx_search_space { argmin_vkey_shift };
    	
	for ( int ii = 1; ii < SEARCH_RADIUS + 1; ii++ ){
        	shift_idx_search_space.push_back( (argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols() );
        	shift_idx_search_space.push_back( (argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols() );
    	}
    	std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

    	// 2. fast columnwise diff 
    	int argmin_shift = 0;
    	double min_sc_dist = 10000000;
    	
	for ( int num_shift: shift_idx_search_space ){
        	Eigen::MatrixXf sc2_shifted = circshift(_sc2, num_shift);
        	double cur_sc_dist = cosineDistance( _sc1, sc2_shifted );
        	
		if( cur_sc_dist < min_sc_dist ){
            		argmin_shift = num_shift;
            		min_sc_dist = cur_sc_dist;
        	}
    	}

    	return std::make_pair(min_sc_dist, argmin_shift);	
}


template<typename T, int Dimension>
const std::pair<int, float> ScanContext<T, Dimension>::detectLoopClosureID()
{
	int loop_id = -1;
	//auto curr_ring_key = ringKeys.back(); // current observation
	Eigen::Matrix<float, Dimension, 1> curr_ring_key = ringKeys.back();
	auto curr_desc = scanContexts.back();	

	// 1. candidates from ringkey tree_ 
	if( ringKeys.size() < NUM_EXCLUDE_RECENT + 1 ){
		std::pair<int, float> ret{ loop_id, 0.0f };
		return ret;
	}

	// 2. kd tree construction
	if( tree_making_period_conter % TREE_MAKING_PERIOD == 0 ){
		std::cout<<" ========================= ReConstruct the KD Tree  ======================"<<std::endl;
		ringKeysMat.clear(); // samples 
		ringKeysMat.assign( ringKeys.begin(), ringKeys.end() - NUM_EXCLUDE_RECENT );
		
		kdTree.reset();
		kdTree = std::make_unique<myKDTree>( Dimension, ringKeysMat, 10 );
		std::cout<<" ========================= Reconstruct the kd tree ======================"<<std::endl;
	}
	tree_making_period_conter += 1;

#ifdef TERMINAL_LOG
	std::cout<<"Tree Making Period Counter : "<<tree_making_period_conter<<std::endl;	
#endif

	float min_dist = 10000000; // init with somthing large
    	int nn_align = 0;
    	int nn_idx = 0;

    	// knn search
    	std::vector<size_t> candidate_indexes( NUM_CANDIDATES_FROM_TREE ); 
   	std::vector<float> out_dists_sqr( NUM_CANDIDATES_FROM_TREE );
	
	nanoflann::KNNResultSet<float> knnsearch_result( NUM_CANDIDATES_FROM_TREE );
    	knnsearch_result.init( &candidate_indexes[0], &out_dists_sqr[0] );
    	kdTree->index->findNeighbors( knnsearch_result, &curr_ring_key[0], nanoflann::SearchParams(10) ); 

	// 2.  pairwise distance (find optimal columnwise best-fit using cosine distance)
	for ( int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++ ){
		Eigen::MatrixXf context_candidate = scanContexts[ candidate_indexes[candidate_iter_idx] ];
			
		std::pair<float, int> sc_dist_result = distanceBetweenScancontexts( curr_desc, context_candidate ); 
		
		float candidate_dist = sc_dist_result.first;
        	int candidate_align = sc_dist_result.second;
		
		if( candidate_dist < min_dist ){
            		min_dist = candidate_dist;
            		nn_align = candidate_align;

            		nn_idx = candidate_indexes[candidate_iter_idx];
        	}
	}

     	// 3. loop threshold check
    	if( min_dist < SC_DIST_THRES ){
        	loop_id = nn_idx;

			std::cout << "---------------------------------------------------------------------------" << std::endl;
			std::cout << "------------------------------- LOOP FOUND --------------------------------" << std::endl;
			std::cout << "---------------------------------------------------------------------------" << std::endl;

			// std::cout.precision(3); 
        	std::cout << "[Loop found] Nearest distance: " << min_dist << " between " << scanContexts.size()-1 << " and " << nn_idx << "." << std::endl;
        	std::cout << "[Loop found] yaw diff: " << nn_align * UNIT_SECTOR_ANGLE << " deg." << std::endl;
		std::cout << "--------------------------------- END ----------------------------------"<<std::endl;

    	}
 /*   	else{
        	std::cout.precision(3); 
        	std::cout << "[Not loop] Nearest distance: " << min_dist << " btn " << scanContexts.size()-1 << " and " << nn_idx << "." << std::endl;
        	std::cout << "[Not loop] yaw diff: " << nn_align * UNIT_SECTOR_ANGLE << " deg." << std::endl;
    	}
*/
    	// To do: return also nn_align (i.e., yaw diff)
    	float yaw_diff_rad = deg2rad<float>(nn_align * UNIT_SECTOR_ANGLE);
    	std::pair<int, float> result {loop_id, yaw_diff_rad};

    	return result;
}

template <typename T, int Dimension>
void ScanContext<T, Dimension>::detectScancontextID( yaw_dist_id &yaw_dist_id_)
{
	int loop_id = -1;
	// auto curr_ring_key = ringKeys.back(); // current observation
	Eigen::Matrix<float, Dimension, 1> curr_ring_key = ringKeys.back();
	auto curr_desc = scanContexts.back();
	ringKeys.pop_back();
	scanContexts.pop_back();

	// 1. candidates from ringkey tree_
	if (ringKeys.size() < NUM_EXCLUDE_RECENT + 1){
		yaw_dist_id_ = yaw_dist_id{  0.0f, 0.0f, loop_id };
		return;
	}

	// 2. kd tree construction
	if (tree_making_period_conter % TREE_MAKING_PERIOD == 0){
		std::cout << " ========================= ReConstruct the KD Tree  ======================" << std::endl;
		ringKeysMat.clear(); // samples
		ringKeysMat.assign(ringKeys.begin(), ringKeys.end() - NUM_EXCLUDE_RECENT);

		kdTree.reset();
		kdTree = std::make_unique<myKDTree>(Dimension, ringKeysMat, 10);
		std::cout << " ========================= Reconstruct the kd tree ======================" << std::endl;
	}
	tree_making_period_conter += 1;

#ifdef TERMINAL_LOG
	std::cout << "Tree Making Period Counter : " << tree_making_period_conter << std::endl;
#endif

	float min_dist = 10000000; // init with somthing large
	int nn_align = 0;
	int nn_idx = 0;

	// knn search
	std::vector<size_t> candidate_indexes(NUM_CANDIDATES_FROM_TREE);
	std::vector<float> out_dists_sqr(NUM_CANDIDATES_FROM_TREE);

	nanoflann::KNNResultSet<float> knnsearch_result(NUM_CANDIDATES_FROM_TREE);
	knnsearch_result.init(&candidate_indexes[0], &out_dists_sqr[0]);
	kdTree->index->findNeighbors(knnsearch_result, &curr_ring_key[0], nanoflann::SearchParams(10));

	// 2.  pairwise distance (find optimal columnwise best-fit using cosine distance)
	for (int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++){
		Eigen::MatrixXf context_candidate = scanContexts[candidate_indexes[candidate_iter_idx]];

		std::pair<float, int> sc_dist_result = distanceBetweenScancontexts(curr_desc, context_candidate);

		float candidate_dist = sc_dist_result.first;
		int candidate_align = sc_dist_result.second;

		if (candidate_dist < min_dist){
			min_dist = candidate_dist;
			nn_align = candidate_align;

			nn_idx = candidate_indexes[candidate_iter_idx];
		}
	}

	// 3. loop threshold check
	if (min_dist < SC_DIST_THRES){
		loop_id = nn_idx;

		std::cout << "---------------------------------------------------------------------------" << std::endl;
		std::cout << "------------------------------- LOOP FOUND --------------------------------" << std::endl;
		std::cout << "---------------------------------------------------------------------------" << std::endl;

		// std::cout.precision(3);
		std::cout << "[Loop found] Nearest distance: " << min_dist << " between " << scanContexts.size() - 1 << " and " << nn_idx << "." << std::endl;
		std::cout << "[Loop found] yaw diff: " << nn_align * UNIT_SECTOR_ANGLE << " deg." << std::endl;
		std::cout << "--------------------------------- END ----------------------------------" << std::endl;
	}
	/*   	else{
			   std::cout.precision(3);
			   std::cout << "[Not loop] Nearest distance: " << min_dist << " btn " << scanContexts.size()-1 << " and " << nn_idx << "." << std::endl;
			   std::cout << "[Not loop] yaw diff: " << nn_align * UNIT_SECTOR_ANGLE << " deg." << std::endl;
		   }
   */
	// To do: return also nn_align (i.e., yaw diff)
	float yaw_diff_rad = deg2rad<float>(nn_align * UNIT_SECTOR_ANGLE);
	yaw_dist_id_ = {yaw_diff_rad, min_dist, loop_id};

}


template<typename T, int Dimension>
void ScanContext<T, Dimension>::displayAScancontext( const Eigen::MatrixXf &desc ) const
{
	cv::Mat image = cv::Mat::zeros( NUM_RING * 10, NUM_SECTOR * 10, CV_8UC3 );

	for( int i = 0; i < NUM_RING; i ++ ){
		cv::line( image, cv::Point( 0, 10 * i ), cv::Point( NUM_SECTOR * 10, 10 * i ), cv::Scalar( 67, 128, 94 ), 1 );
	}

	for( int i = 0; i < NUM_SECTOR; i ++ ){
		cv::line( image, cv::Point( 10 * i, 0 ), cv::Point( 10 * i, NUM_RING * 10 ), cv::Scalar( 67, 128, 94 ), 1 );
	}

	for( int i = 0; i < desc.rows(); i ++ ){
		for( int j = 0; j < desc.cols(); j ++ ){
			if( desc( i, j ) > 0 ){
				//std::cout<<"point: ( "<<i <<", "<<j <<" )"<<std::endl;
				cv::Point points[1][4];
				points[0][0] = cv::Point(j * 10, i * 10);
				points[0][1] = cv::Point(j * 10 + 10, i * 10);
				points[0][2] = cv::Point(j * 10 + 10, i * 10 + 10);
				points[0][3] = cv::Point(j * 10, i * 10 + 10);				
				
				const cv::Point* ppt[1] = { points[0] };
				int npt[] = { 4 };

				int count = static_cast<int>( desc( i, j ) );
				cv::fillPoly(image, ppt, npt, 1, cv::Scalar(b[64 - count], g[64 - count], r[64 - count]));
			}
		}
	}

	cv::imshow( "scancontext", image );
	
}

template<typename T, int Dimension>
void ScanContext<T, Dimension>::displayScanDistribution( const slam::sensor::LaserScan &scan ) 
{
	cv::Mat image = cv::Mat::zeros( 900, 900, CV_8UC3 );

	for( int i = 0; i < NUM_RING; i ++ ){
		cv::circle( image, cv::Point2f( 450, 450 ), 20 * ( i + 1 ), cv::Scalar( 0, 255, 0 ), 1 );
	}

	for( int i = 0; i < NUM_SECTOR; i ++ ){
		float angle =  ( 6 * i );	
	
		cv::Point2f endPoint( 450 - ::sin( angle * M_PI / 180 ) * 400, 
				      450 - ::cos( angle * M_PI / 180 ) * 400 );
		cv::line( image, cv::Point2f( 450, 450 ), endPoint, cv::Scalar( 0, 255, 0 ), 1 );
	}

        cv::arrowedLine( image, cv::Point2f( 450, 900 ), cv::Point2f( 450, 20 ), cv::Scalar( 255, 0, 0 ), 1 );
        cv::arrowedLine( image, cv::Point2f( 0, 450 ), cv::Point2f( 880, 450 ), cv::Scalar( 255, 0, 0 ), 1 );

	
	int ring_idx = 0, sctor_idx = 0;
        float radians = -3.14159f;

        for( int i = 0; i < scan.size(); i ++ ){
                float dist = scan.ranges[i];
                float angle = rad2deg<float>( radians ) + 180.0f;

                if( dist >= 0.009999998f && dist <= 20.0000000000f ){
                        ring_idx = std::max( std::min( NUM_RING - 1, int(ceil( (dist / MAX_RADIUS) * NUM_RING )) ), 0 );

                        sctor_idx = std::max( std::min( NUM_SECTOR - 1, int(ceil( (angle / 360.0) * NUM_SECTOR )) ), 0 );

			// draw the bin
			//cv::Point2f binPoint( 450 - ( ring_idx * 20 ) * ::sin( sctor_idx * 6 * M_PI / 180 ) + 10, 
			//		      450 - ( ring_idx * 20 ) * ::cos( sctor_idx * 6 * M_PI / 180 ) - 10 );
			//cv::circle( image, binPoint, 3, cv::Scalar( 0, 0, 255 ), -1 );
                
			drawABin( image, ring_idx, sctor_idx );
		}

                radians += 0.0043542264f;
        }

	
	cv::imshow( "scan distribution", image );
}

template<typename T, int Dimension>
const Eigen::MatrixXf& ScanContext<T, Dimension>::getScanContext( int index ) const
{
	return scanContexts[index];
}

template<typename T, int Dimension>
const Eigen::MatrixXf& ScanContext<T, Dimension>::getRingKey( int index ) const
{
	return ringKeys[ index ];
}

template<typename T, int Dimension>
const int ScanContext<T, Dimension>::getScanContextsSize() const
{
	return scanContexts.size();
}


template<typename T, int Dimension>
void ScanContext<T, Dimension>::drawABin(cv::Mat &image,  int ring_idx, const int sctor_idx)
{
	float angleStart1 = (sctor_idx) * 6 * M_PI / 180;
	float radiusStart1 = (ring_idx) * 20;
	float angleStart2 = (sctor_idx + 1) * 6 * M_PI / 180;
	float radiusStart2 = (ring_idx + 1) * 20;

	std::vector<std::vector<cv::Point>> ppPoints;
	std::vector<cv::Point> pPoints;

	for (int i = 0; i <= 6; i++) {
		cv::Point p( 450 - radiusStart1 * sin(angleStart1 + i * 0.017453293),
			     450 - radiusStart1 * cos(angleStart1 + i * 0.017453293));
		pPoints.push_back( p );
	}

	for (int i = 0; i <= 6; i++) {
		cv::Point p( 450 - radiusStart2 * sin( angleStart2 - i * 0.017453293),
			     450 - radiusStart2 * cos( angleStart2 - i * 0.017453293));
		pPoints.push_back( p );
	}

	ppPoints.push_back( pPoints );

	/*for (auto it : pPoints) {
		cv::circle(image, it, 3, cv::Scalar(0, 0, 255), -1);
		cv::imshow("scan distribution", image);
		cv::waitKey(2000);
	}*/

	cv::fillPoly( image, ppPoints, cv::Scalar( 0, 0, 255 ) );
	//cv::fillPoly( image, ppPoints, cv::Scalar(b[64 - count], g[64 - count], r[64 - count]) );
}

}

#endif
