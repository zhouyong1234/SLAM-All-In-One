#ifndef __TIME_MANAGE_H_
#define __TIME_MANAGE_H_

#include <iostream>
#include <chrono>

namespace slam{

class Time
{
public:
	static void begin();
	static void end();

	static const double duration_s();
	static const double duration_ms();
	static const double duration_ns();

private:
	static std::chrono::steady_clock::time_point beginTimePoint;
	static std::chrono::steady_clock::time_point endTimePoint;

};

std::chrono::steady_clock::time_point Time::beginTimePoint = std::chrono::steady_clock::time_point();
std::chrono::steady_clock::time_point Time::endTimePoint = std::chrono::steady_clock::time_point();


}

#endif
