#include "slamProcessor.h"
#include "laserSimulation.h"

#include "odomSimulation.h"

#include "loopClosureScanContext.h"

#include "graphOptimize.h"

#include <unistd.h>

void laserData2Container( const slam::sensor::LaserScan &scan, slam::ScanContainer &container )
{
        size_t size = 1440;

        float angle = -3.14159f;
        container.clear();

        for( int i = 0; i < size; i ++ ){
                float dist = scan.ranges[ i ];

                if( dist >= 0.0099999998f && dist <= 15.0000000000f ){
                        //dist *= scaleToMap;
                        container.addData( Eigen::Vector2f( cos(angle) * dist, sin(angle) * dist ) );
                }

                angle += 0.0043633231f;
        }

        std::cout<<"Scan Container Size: "<<container.getSize()<<std::endl;
}

bool poseDiffLargerThan( Eigen::Vector3f &poseOld, Eigen::Vector3f &poseNew )
{

        if( ( ( poseNew.head<2>() - poseOld.head<2>() ).norm() ) > 0.4f ){
                return true;
        }

        float angleDiff = ( poseNew.z() - poseOld.z() );

        if( angleDiff > M_PI ){
                angleDiff -= M_PI * 2.0f;
        }
        else if( angleDiff < -M_PI ){
                angleDiff += M_PI * 2.0f;
        }

        if( ::abs( angleDiff ) > 0.9f ){
                return true;
        }

        return false;
}


Eigen::Matrix<float, 3, 3> v2t(Eigen::Vector3f &v)
{
	float c = ::cos( v(2) );
	float s = ::sin( v(2) );

	Eigen::Matrix<float, 3, 3> A;
	A << c, -s, v(0),
	     s,  c, v(1),
	     0,  0,  1;

	return A;
}

Eigen::Vector3f t2v(Eigen::Matrix<float, 3, 3> &A)
{
	Eigen::Vector3f v;
	
	v(0) = A(0, 2);
	v(1) = A(1, 2);
	v(2) = ::atan2( A( 1, 0 ), A(0, 0) );

	return v;
}

int main()
{
	std::cout<<"--------------------- G2O Test --------------------"<<std::endl;
	
	slam::simulation::OdomSimulation odomSim;

	// g2o instance
        slam::optimizer::GraphOptimize optimizer;

	optimizer.createOptimizer();

	optimizer.setMaxIeration(10);

	std::string odom_file_name = "../../../../simulation_file/odometry2.txt";
 	if( !odomSim.openSimulationFile( odom_file_name ) ){
		//std::cout<<"Open File Failed ..."<<std::endl;
		//return 0;
	}

	Eigen::Vector3f poseOld = Eigen::Vector3f::Zero();
        Eigen::Vector3f poseNew = poseOld;

        std::vector<Eigen::Vector3f> keyPoses;

	int keyCount = -1;


	while( !odomSim.endOfInputFile() ){
		odomSim.readAFrameData( poseNew );
		std::cout<<"frame count: "<<odomSim.getFrameCount()<<std::endl;
  //              std::cout<<"pose: "<<std::endl<<poseNew<<std::endl;

		if( poseDiffLargerThan( poseOld, poseNew ) ){
                        std::cerr<<"------------------ UPDATE ----------------"<<std::endl;
			//std::cout<<"pose: "<<std::endl<<poseNew<<std::endl;
			keyCount ++;	

			if( keyCount == 0 ){
				std::cout<<"keyCount = "<<keyCount <<std::endl;
				std::cout<<"pose new: "<<std::endl<<poseNew<<std::endl;

                                keyPoses.push_back( poseNew );

				optimizer.addVertex( poseNew, keyCount ); // add a vertex
			}
			else{
        	                std::cout<<"keyCount = "<<keyCount <<std::endl;
                	        keyPoses.push_back( poseNew );
				
				optimizer.addVertex( poseNew, keyCount ); // add a vertex
				std::cout<<"pose new: "<<std::endl<<poseNew<<std::endl;				
				// -------------------------------------------//
		
				//Eigen::Vector3f poseDiff = poseNew - poseOld;
				//std::cout<<"Pose Diff: "<<std::endl<<poseDiff<<std::endl;				
				
				Eigen::Matrix<float, 3, 3> T1 = v2t( poseOld );
				Eigen::Matrix<float, 3, 3> T2 = v2t( poseNew );

				Eigen::Matrix<float, 3, 3> T = T1.inverse() * T2;
				Eigen::Vector3f V = t2v( T );
				std::cout<<"V = "<<std::endl<<V<<std::endl<<std::endl;	
				//-------------------- END ----------------------//

				Eigen::Matrix3d information = 1 * Eigen::Matrix3d::Identity(); //information matrix
				//std::cout<<"information matrix: "<<std::endl<<information<<std::endl;
				std::cout<<"edge: "<<keyCount - 1<<" to "<<keyCount<<std::endl;
                	        optimizer.addEdge( V, keyCount - 1, keyCount, information ); // add a edge
			}

			//if( keyCount % 130 == 0 && keyCount != 0 ){
                        if( keyCount == 127 ){
				int loopId = 34;		
				std::cout<<"loop pose new : "<<std::endl<<poseNew<<std::endl<<std::endl;				
				std::cout<<"loop pose old : "<<std::endl<<keyPoses[loopId]<<std::endl<<std::endl;

				Eigen::Matrix3d information = 1 * Eigen::Matrix3d::Identity(); //information matrix
				Eigen::Matrix<float, 3, 3> T1 = v2t( poseNew );
                                Eigen::Matrix<float, 3, 3> T2 = v2t( keyPoses[loopId] );

                                Eigen::Matrix<float, 3, 3> T = T1.inverse() * T2;
                                Eigen::Vector3f V = t2v( T );

				optimizer.addEdge( V, keyCount, loopId, information );
				std::cout<<"Add A Loop Correlation ..."<<std::endl;			
				
				std::cout<<"----------- execuate the graph optimization ----------"<<std::endl;

                                optimizer.execuateGraphOptimization();

                                optimizer.getOptimizedResults();

                                std::vector<Eigen::Vector3f> estimatedPoses = optimizer.getEstimatedPoses();
                                std::cout<<"keyPoses.size  = "<<keyPoses.size()<<std::endl;
                                std::cout<<"estimatedPoses.size = "<<estimatedPoses.size()<<std::endl;
                                // TODO ... process the estimated results

				break;
                        }


			poseOld = poseNew;
		}		
		
		cv::waitKey( 60 );
	}

	odomSim.closeSimulationFile();

	return 0;
}
