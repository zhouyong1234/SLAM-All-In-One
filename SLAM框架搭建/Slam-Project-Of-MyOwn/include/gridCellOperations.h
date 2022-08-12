#ifndef __GRID_CELL_OPERATIONS_H_
#define __GRID_CELL_OPERATIONS_H_

#include "gridCell.h"

namespace slam {

class GridCellOperations
{
public:
	GridCellOperations();
	~GridCellOperations();

	const float getCellProbability( const GridCell &cell ) const;
	
	void setLogOddsPocc( float Pocc );
	void setLogOddsPfree( float Pfree );

	const float getLogOddsPocc() const;
	const float getLogOddsPfree() const;

	void setCellFree( GridCell &cell ) const;
	void setCellOccupied( GridCell &cell ) const;
	void setCellUnFree( GridCell &cell ) const;
	
private:
	const float probability2LogOdds( float prob );

private:
	float logOddsPocc;
	float logOddsPfree;
	
};

}

#endif
