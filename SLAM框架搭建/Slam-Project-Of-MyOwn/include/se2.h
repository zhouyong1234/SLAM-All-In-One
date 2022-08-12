#ifndef __SE2_H_
#define __SE2_H_

#include "g2o/config.h"

#include "g2o/stuff/misc.h"
#include "g2o/stuff/macros.h"

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace slam{

namespace optimizer{

class SE2
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	
	SE2() : _R(0), _t(0,0)
	{

	}

	SE2( double x, double y, double theta ) : _R( theta ), _t( x, y ) 
	{

	}
	
	const Eigen::Vector2d& translation() const 
	{
		return _t;
	}

	Eigen::Vector2d& translation() 
	{
		return _t;
	}

	const Eigen::Rotation2Dd& rotation() const 
	{
		return _R;
	}

        Eigen::Rotation2Dd& rotation() 
	{
		return _R;
	}

	SE2 operator * (const SE2& tr2) const
	{
          	SE2 result( *this );
          
		result._t += _R * tr2._t;
          	result._R.angle() += tr2._R.angle();
          	result._R.angle() = g2o::normalize_theta( result._R.angle() );
          
		return result;
        }

	SE2& operator *= (const SE2& tr2)
	{
         	_t += _R * tr2._t;
          	_R.angle() += tr2._R.angle();
          	_R.angle() = g2o::normalize_theta( _R.angle() );
          
		return *this;
        }

	Eigen::Vector2d operator * (const Eigen::Vector2d& v) const 
	{
          	return _t + _R * v;
        }

	SE2 inverse() const
	{
          	SE2 ret;
          	ret._R = _R.inverse();
          	ret._R.angle() = g2o::normalize_theta( ret._R.angle() );
          	ret._t = ret._R * ( Eigen::Vector2d( -1 * _t ) );
          
		return ret;
        }

	double operator [](int i) const 
	{
          	assert (i >= 0 && i < 3);
        
	  	if (i < 2)
            		return _t(i);
          
		return _R.angle();
        }

	double& operator [](int i) 
	{
          	assert (i >= 0 && i < 3);
	
          	if (i < 2)
            		return _t( i );
          		
		return _R.angle();
        }

        void fromVector (const Eigen::Vector3d& v)
	{
          	*this = SE2(v[0], v[1], v[2]);
        }

        Eigen::Vector3d toVector() const 
	{
          	Eigen::Vector3d ret;
          	
		for (int i = 0; i < 3; i ++){
            		ret(i) = ( *this )[i];
          	}
          
		return ret;
        }

protected:
	Eigen::Rotation2Dd _R;
        Eigen::Vector2d _t;

};

}

}

#endif
