#include "time_manage.h"

namespace slam{


void Time::begin()
{
	beginTimePoint = std::chrono::steady_clock::now();
}

void Time::end()
{
	endTimePoint = std::chrono::steady_clock::now();
}

const double Time::duration_s()
{
	return std::chrono::duration<double>( endTimePoint - beginTimePoint ).count();
}

const double Time::duration_ms()
{
	return std::chrono::duration<double, std::milli>(endTimePoint - beginTimePoint).count();
}

const double Time::duration_ns()
{
	return std::chrono::duration<double, std::micro>(endTimePoint - beginTimePoint).count();
}

}
