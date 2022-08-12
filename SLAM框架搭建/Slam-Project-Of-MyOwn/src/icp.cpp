#include "icp.h"
#include <cmath>
#include <iostream>
#include <numeric>

namespace slam{

ICP::ICP()
{

}

ICP::~ICP()
{

}

// B -> A
const float ICP::solveICP( ScanContainer& A_src, ScanContainer& B_src )
{
	A.clear();
	std::vector<Eigen::Vector2f> B; 

	for( size_t i = 0; i < A_src.getSize(); i ++ ){
		A.push_back( A_src.getIndexData(i) );
	}	
	
	for( size_t i = 0; i < B_src.getSize(); i ++ ){
                B.push_back( B_src.getIndexData(i) );
        }

	// 1. Caculate the center of point cloud of A
	// A_center = ( A_1 + A_2 + ...A_n ) / n;
	Eigen::Vector2f Asum(0.0f, 0.0f);
	//for( auto it : A ){
	//	Asum += it;
	//}
	Asum = std::accumulate( A.begin(), A.end(), Eigen::Vector2f( 0.0f, 0.0f ), []( Eigen::Vector2f &a, Eigen::Vector2f &b ){ return a + b; } );	

	Acenter = Asum / A.size();
	std::cout<<"A center: "<<std::endl<<Acenter<<std::endl<<std::endl;
	

	// 2. Caculate the distance from points to the center of A
	// a_i = A_i - A_center, i = 1, 2, ...., n
	a.resize( A.size(), Eigen::Vector2f( 0.0f, 0.0f ) );
	for( size_t i = 0; i < A.size(); i ++){
		a[i] = A[i] - Acenter;
	}
	
	float initial_loss = cacuLoss( B );
	std::cout<<"initial loss is: "<<initial_loss<<std::endl;
	std::cout<<"-----------------------------------"<<std::endl;	

	// begin iterate
	int iteration = 0;
	float loss_now = 0.0f;
	float loss_improve = 0.0f;
	float loss_before = 0.0f;
	
	// Error Function : 
	// E(R, T) = Sigma(i = 1 to n){a_i * R * b_i} / n

	// E(R, T) = Sigma(i = 1 to n){ cos(theta) * (a_i(x) * b_i(x) + a_i(y) * b_i(y)) + sin(theta) * (a_i(x) * b_i(y) - a_i(y) * b_i(x)) } / n

	// dE(R, T) / d(theta) = Sigma(i = 1 to n){-sin(theta) * (a_i(x) * b_i(x) + a_i(y) * b_i(y)) + cos(theta) * ( a_i(x) * b_i(y) - a_i(y) * b_i(x) ) } / n

	// let dE(R, T) / d(theta) = 0

	// tan(theta) = Sigma(i = 1 to n){( a_i(x) * b_i(y) - a_i(y) * b_i(x) )} / Sigma(i = 1 to n){ (a_i(x) * b_i(x) + a_i(y) * b_i(y)) }
	
	while( iteration < maxIteration ){
		loss_now = iterateOnce( B );
		loss_improve = loss_before - loss_now;	
		loss_before = loss_now;	

		iteration ++;
		
		std::cout<<"lose improve = "<<loss_improve<<std::endl;
		std::cout<<"-------------------------"<<std::endl;
	}

	return loss_improve;
}

const float ICP::iterateOnce( std::vector<Eigen::Vector2f>& B )
{
	Eigen::Vector2f Bsum(0.0f, 0.0f);

	//for( auto it : B ){
	//	Bsum += it;
	//}
	Bsum = std::accumulate( B.begin(), B.end(), Eigen::Vector2f( 0.0f, 0.0f ), []( Eigen::Vector2f &a, Eigen::Vector2f &b ){ return a + b; } );
	Bcenter = Bsum / B.size();
	std::cout<<"B center: "<<std::endl<<Bcenter<<std::endl<<std::endl;


	b.resize( B.size(), Eigen::Vector2f( 0.0f, 0.0f ) );
	for( size_t i = 0; i < B.size(); i ++ ){
		b[i] = B[i] - Bcenter;
	}

	// 3. caculate the rotate theta
	// theta = arctan( Sigma(i = 1 to n){( a_i(y) * b_i(x) - a_i(x) * b_i(y) )} / Sigma(i = 1 to n){ (a_i(x) * b_i(x) + a_i(y) * b_i(y)) } );
	float y = 0.0f, x = 0.0f;
	
	for( size_t i = 0; i < b.size(); i ++ ){
		// find the closest point index in a set
		int index = getClosestPointID( b[i], a );	
	
		//  1        | ( a_i_y * b_i_x - a_i_x * b_i_y ) | 
		// --- Sigma | --------------------------------- | = tan( theta )
		//  n        | ( a_i_x * b_i_x + a_i_y * b_i_y ) |
		//y += ( a[index](1) * b[i](0) ) - ( a[index](0) * b[i](1) );
		y += ( a[index](0) * b[i](1) ) - ( a[index](1) * b[i](0) );		

		//y += ( a[i](0) * b[index](1) ) - ( a[i](1) * b[index](0) );		
		x += ( a[index](0) * b[i](0) ) + ( a[index](1) * b[i](1) );
	}
	float theta = ::atan2( y, x );
	std::cout<<"theta = "<<theta<<std::endl;	

	// 4. get the rotate matrix and transfrom matrix
	//	R = |cos(theta), -sin(theta)|
	//	    |sin(theta),  cos(theta)|
	R( 0, 0 ) = ::cos(theta);
	R( 0, 1 ) = -::sin(theta);
	R( 1, 0 ) = ::sin(theta);
	R( 1, 1 ) = ::cos(theta);
	std::cout<<"R = "<<std::endl<<R<<std::endl<<std::endl;
	

	// T = A_center - R * B_center
	T = Acenter - R * Bcenter;
	std::cout<<"T = "<<std::endl<<T<<std::endl<<std::endl;

	// 5. get new pose B' from B
	// B_apostrophe_i = R * B_i + T
	for( size_t i = 0; i < B.size(); i ++ ){
		Eigen::Vector2f tmp = R * B[i] + T;
		B[i] = tmp;;
	}

	// 6. caculate the pose differential
	R_final = R * R_final;
	T_final = R * T_final + T;	

	std::cout<<"R_final = "<<std::endl<<R_final<<std::endl;
	std::cout<<"T_final = "<<std::endl<<T_final<<std::endl;

	// 7. caculate loss function
	// Loss = Sigma(i = 1 to n){(A_i - B_apostrophe_i) * (A_i - B_apostrophe_i)} / n
	return cacuLoss( B );
}

const float ICP::cacuLoss( const std::vector<Eigen::Vector2f> &B )
{
	float loss = 0.0f;
        for( size_t i = 0; i < B.size(); i ++ ){
                int index = getClosestPointID( A[i], B );

                Eigen::Vector2f tmp = A[i] - B[index];
                loss += tmp.norm();
        }
        loss /= B.size();

	std::cout<<"lose = "<<loss<<std::endl<<std::endl;
        return loss;	
}

const Eigen::Vector2f ICP::getClosestPoint( const Eigen::Vector2f &point, const std::vector<Eigen::Vector2f> &sourcePoints )
{
	float dist_min = 100000.0f;
	Eigen::Vector2f closestPoint;	

	for( int i = 0; i < sourcePoints.size(); i ++ ){
		float dist = ( point - sourcePoints[i] ).norm();
		
		if( dist < dist_min ){
			dist_min = dist;
			
			closestPoint(0) = sourcePoints[i](0);
			closestPoint(1) = sourcePoints[i](1);
		}
	}

	return closestPoint;
}

const int ICP::getClosestPointID( const Eigen::Vector2f &point, const std::vector<Eigen::Vector2f> &sourcePoints )
{
	float dist_min = 100000.0f;
	int id = 0;
	
	for( int i = 0; i < sourcePoints.size(); i ++ ){
		float dist = ( point - sourcePoints[i] ).norm();
	
		if( dist < dist_min ){
			dist_min = dist;
			
			id = i;
		}
	}

	return id;

}



}






