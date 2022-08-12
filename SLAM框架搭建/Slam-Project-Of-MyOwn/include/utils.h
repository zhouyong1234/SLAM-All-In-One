#ifndef __UTILS_H_
#define __UTILS_H_

#include <cmath>

namespace slam {

static inline float normalize_angle_pos(float angle)
{
	return fmod( fmod( angle, 2.0f * M_PI ) + 2.0f * M_PI, 2.0f * M_PI );
}

static inline float normalize_angle(float angle)
{
	float a = normalize_angle_pos(angle);

	if (a > M_PI){
    		a -= 2.0f*M_PI;
  	}

  	return a;
}

template<typename T>
static T toDeg(const T radVal)
{
	return radVal * static_cast<T>(180.0 / M_PI);
}

template<typename T>
static T toRad(const T degVal)
{
	return degVal * static_cast<T>(M_PI / 180.0);
}

static inline float sqr(float val)
{
  return val*val;
}

}

#endif
