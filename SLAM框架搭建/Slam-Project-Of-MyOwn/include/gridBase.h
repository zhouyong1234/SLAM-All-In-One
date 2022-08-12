#ifndef __GRID_BASE_H_
#define __GRID_BASE_H_

#include "gridCell.h"
#include "gridCellOperations.h"
#include "gridMapBase.h"

namespace slam {

typedef GridMapBase<GridCell, GridCellOperations> GridBase;


}

#endif

