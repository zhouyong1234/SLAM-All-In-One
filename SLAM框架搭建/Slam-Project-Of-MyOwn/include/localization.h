#ifndef __LOCALIZATION_H_
#define __LOCALIZATION_H_

#include <iostream>
#include "laserSimulation.h"

#include "scanContext.h"
#include "scanContainer.h"

#include "odomSimulation.h"

#include "readScanContext.h"

#include "loadMap.h"
#include "localization.h"
#include "slamProcessor.h"


namespace slam{

class Localization
{
public:
	Localization();
	~Localization();

private:
	localization::ReadScanContext *readScanContext;
	
};

}

#endif
