#include "StateEstimator.h"

StateEstimator::StateEstimator()
{

}

StateEstimator::~StateEstimator()
{

}

void StateEstimator::initialize()
{
	poses.resize(WINDOW_SIZE);
}
