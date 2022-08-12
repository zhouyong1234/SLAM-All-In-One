#ifndef __Data_Type_H_
#define __Data_Type_H_

#include<string.h>

namespace slam{

namespace sensor{

template<int Size>
struct LidarScan{
	LidarScan() : angle_min(3.12414),
			angle_max(3.14159),
			angle_increment(0.0174532924),
			time_increment(0.0004022129),
			range_min(0.1500000060),
			range_max(12.0000000000)		
	{
		memset( this->ranges, 0, Size );
		memset( this->intensities, 0, Size );
	}

	~LidarScan(){}

	LidarScan( const LidarScan& obj ) : angle_min( obj.angle_min ),
					    angle_max( obj.angle_max ),
					    angle_increment( obj.angle_increment ),
					    scan_time( obj.scan_time ),
					    time_increment( obj.time_increment ),
					    range_min( obj.range_min ),
					    range_max( obj.range_max )
	{
		memcpy( this->ranges, obj.ranges, Size );
		memcpy( this->intensities, obj.intensities, Size );
	}

	LidarScan& operator=( const LidarScan& other )
	{
		if( &other == this )
			return *this;

		angle_min = other.angle_min;
		angle_max = other.angle_max;
		angle_increment = other.angle_increment;
		scan_time = other.scan_time;
		time_increment = other.time_increment;
		range_min = other.range_min;
		range_max = other.range_max;
		memcpy( this->ranges, other.ranges, Size );
                memcpy( this->intensities, other.intensities, Size );
	
		return *this;
	}

	void setParameters( const float angle_min, 
			    const float angle_max, 
			    const float angle_increment,
			    const float range_min,
			    const float range_max, 
			    const float scan_time = 0,
			    const float time_increment = 0 )
	{
		this->angle_min = angle_min;
		this->angle_max = angle_max;
		this->angle_increment = angle_increment;
		this->range_min = range_min;
		this->range_max = range_max;
		this->scan_time = scan_time;
		this->time_increment = time_increment;
	}

	const int size()
	{
		return Size;
	}

	const int size() const{
		return Size;
	}
	
	float angle_min;
        float angle_max;
        float angle_increment;
        float scan_time;
        float time_increment;
        float range_min;
        float range_max;
        float ranges[Size];
        float intensities[Size];
	
};

typedef struct LidarScan<1440> LaserScan;

struct IMU
{
	IMU() : ax( 0.0f ),
		ay( 0.0f ),
		az( 0.0f ),
		gx( 0.0f ),
		gy( 0.0f ),
		gz( 0.0f )
	{

	}	

	IMU( 	float ax_, 
		float ay_, 
		float az_, 
		float gx_, 
		float gy_, 
		float gz_ ) : ax( ax_ ),
			     ay( ay_ ),
			     az( az_ ),
			     gx( gx_ ),
			     gy( gy_ ),
			     gz( gz_ )
	{

	}
	
	IMU( const IMU &rhs ) : ax( rhs.ax ),
				ay( rhs.ay ),
				az( rhs.az ),
				gx( rhs.gx ),
				gy( rhs.gy ),
				gz( rhs.gz )
	{

	}

	const IMU& operator=( const IMU &rhs )
	{
		if( &rhs == this ){
			return *this;
		}

		ax = rhs.ax;
		ay = rhs.ay;
		az = rhs.az;
		gx = rhs.gx;
		gy = rhs.gy;
		gz = rhs.gz;

		return *this;
	}	

	~IMU()
	{

	}	

	float ax;// x 方向加速度
	float ay;// y 方向加速度
	float az;// z 方向加速度
	float gx;// x 方角速度
	float gy;// y 方角速度
	float gz;// z 方角速度
};

typedef struct IMU IMU;

}

}

#endif
