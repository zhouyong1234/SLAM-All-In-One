#include "saveAsBMP.h"

namespace slam{

bool SaveAsBMP::operator()( const std::string &fileName, OccupiedMap &map )
{
	cv::Mat image = cv::Mat(map.getSizeX(), map.getSizeY(), CV_8UC1, cv::Scalar::all(125));

	for( int i = 0; i < map.getSizeX(); i ++ ){
                for( int j = 0; j < map.getSizeY(); j ++ ){
			if( map.isCellFree( i, j ) ){ // free for 255, white
				image.at<uchar>(i, j) = 255;
			}
			else if( map.isCellOccupied( i, j ) ){
				image.at<uchar>(i, j) = 0; // occupied for 0, black
			}
		}
	}

	cv::imshow( "bmp map", image );

	cv::imwrite( fileName, image );
	
}

}
