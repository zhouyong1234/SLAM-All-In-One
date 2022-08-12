#ifndef __SAVE_AS_BMP_H_
#define __SAVE_AS_BMP_H_

#include "occupiedMap.h"

#include <opencv2/opencv.hpp>

namespace slam{

class SaveAsBMP
{
public:
	bool operator()( const std::string &fileName, OccupiedMap &map );

};

}

#endif
