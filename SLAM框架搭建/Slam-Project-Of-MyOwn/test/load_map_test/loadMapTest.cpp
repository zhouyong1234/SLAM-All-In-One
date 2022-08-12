#include <unistd.h>

#include "loadMap.h"

void displayMap( slam::OccupiedMap &occupiedGridMap, cv::Mat &image )
{

        int occupiedCount = 0;

        // display the map
        for( int i = 0; i < occupiedGridMap.getSizeX(); i ++ ){
                for( int j = 0; j < occupiedGridMap.getSizeY(); j ++ ){
                        if( occupiedGridMap.isCellFree( i, j ) ){
                                cv::circle(image, cv::Point2d(i, j), 1, cv::Scalar(255, 255, 255), -1);
				
                        }
                        else if( occupiedGridMap.isCellOccupied( i, j ) ){
                                occupiedCount ++;
                                cv::circle(image, cv::Point2d(i, j), 1, cv::Scalar(0, 0, 255), -1);
                        
		
				std::cout<<"cell( "<<i<<", "<<j <<" ): "<<occupiedGridMap.getCellLogOdds(i, j)<<std::endl;
                	}
		}
        }
 
        std::cout<<"---------------- Result --------------------"<<std::endl;
        std::cout<<"Occupied Points Number: "<<occupiedCount<<std::endl;
}


int main()
{
	std::cout<<"--------------------- Load Map Test --------------------"<<std::endl;

	// 1. instance of the OccupiedMap
	slam::OccupiedMap occupiedMap;
	
	// 2. read the Occupied Map from the file
	occupiedMap = slam::LoadMap()( "../../../simulation_file/test.map" );

	occupiedMap.setMapInfo( 1001,  1001, 10);
	// 3. print the information of the map
	std::cout<<"-------------------- Map Information -----------------"<<std::endl;
	std::cout<<"map sizeX       : "<<occupiedMap.getSizeX()<<std::endl;	
	std::cout<<"map sizeY       : "<<occupiedMap.getSizeY()<<std::endl;
	std::cout<<"map cell length : "<<occupiedMap.getCellLength()<<std::endl;
	std::cout<<"map scale       : "<<occupiedMap.getScale()<<std::endl;
	std::cout<<"------------------------- END -----------------------"<<std::endl;
	
	// init the image
	cv::Mat image = cv::Mat::zeros(occupiedMap.getSizeX(), occupiedMap.getSizeY(), CV_8UC3);

	displayMap( occupiedMap, image );

	cv::imshow( "map", image );

	cv::waitKey(0);	

	return 0;
}
