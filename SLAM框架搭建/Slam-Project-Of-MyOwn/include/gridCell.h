#ifndef __GRID_CELL_H_
#define __GRID_CELL_H_

namespace slam{

class GridCell
{
public:
	GridCell();
	~GridCell();
	GridCell( float logOddsValue_, int updateIndex_ );
	
	void setLogOddsValue( float logOddsValue );

	const float getLogOddsValue() const;

	bool isOccupied() const ;
	bool isFree() const;
	bool isUnknow() const;	

	void resetGridCell();

public:
	float logOddsValue;
	int updateIndex;
};

}

#endif
