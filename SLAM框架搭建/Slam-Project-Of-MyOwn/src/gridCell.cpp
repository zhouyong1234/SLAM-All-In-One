#include "gridCell.h"

namespace slam {

GridCell::GridCell() : logOddsValue(0.0f), updateIndex( 0 )
{

}

GridCell::GridCell( float logOddsValue_, int updateIndex_ ) : logOddsValue(logOddsValue_), updateIndex( updateIndex_ )
{

}


GridCell::~GridCell()
{

}

void GridCell::setLogOddsValue( float logOddsValue )
{
	this->logOddsValue = logOddsValue;
}

const float GridCell::getLogOddsValue() const
{
	return logOddsValue;
}

bool GridCell::isOccupied() const
{
	return ( logOddsValue > 0.0f );
}

bool GridCell::isFree() const
{
	return ( logOddsValue < 0.0f );
}

bool GridCell::isUnknow() const
{
	return ( logOddsValue == 0.0f );
}

void GridCell::resetGridCell()
{
	logOddsValue = 0.0f;
	updateIndex = 0;
}

}
