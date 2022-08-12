#include "gridCellOperations.h"

#include <cmath>
#include <iostream>

namespace slam {

GridCellOperations::GridCellOperations()
{
	logOddsPocc = probability2LogOdds( 0.6f );
	logOddsPfree = probability2LogOdds( 0.4f );

	//------------ for testing ------------//
	std::cout<<"--------- Testing in class GridCellOperations --------"<<std::endl;
	std::cout<<"logOddsPocc = "<<logOddsPocc<<std::endl;
	std::cout<<"logOddsPfree = "<<logOddsPfree<<std::endl;
}

GridCellOperations::~GridCellOperations()
{

}

const float GridCellOperations::probability2LogOdds( float prob )
{
	float odds = prob / ( 1.0f - prob );
	
	return ::log( odds );
}

const float GridCellOperations::getCellProbability( const GridCell &cell ) const
{
	float odds = ::exp( cell.logOddsValue );
	
	//return ( 1.0f - ( 1.0f / ( odds + 1.0f ) ) );
	return ( odds / ( odds + 1.0f ) );
}

void GridCellOperations::setLogOddsPocc( float Pocc )
{
	this->logOddsPocc = probability2LogOdds( Pocc );
}

void GridCellOperations::setLogOddsPfree( float Pfree )
{
	this->logOddsPfree = probability2LogOdds( Pfree );
}

void GridCellOperations::setCellFree( GridCell &cell ) const
{
	cell.logOddsValue += logOddsPfree;
}

void GridCellOperations::setCellOccupied( GridCell &cell ) const
{
	cell.logOddsValue += logOddsPocc;
}

void GridCellOperations::setCellUnFree( GridCell &cell ) const
{
	cell.logOddsValue -= logOddsPfree;
}

const float GridCellOperations::getLogOddsPocc() const
{
	return logOddsPocc;
}

const float GridCellOperations::getLogOddsPfree() const
{
	return logOddsPfree;
}

}
