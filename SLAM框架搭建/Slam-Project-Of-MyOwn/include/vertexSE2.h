#ifndef __SLAM_VERTEX_SE2_H_
#define __SLAM_VERTEX_SE2_H_

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "se2.h"

namespace slam{

namespace optimizer{

// 2D pose Vertex, (x,y,theta)
class VertexSE2 : public g2o::BaseVertex<3, SE2>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	VertexSE2();

	virtual void setToOriginImpl() 
	{
        	_estimate = SE2();
        }

	virtual void oplusImpl(const double* update)
        {
          	SE2 up(update[0], update[1], update[2]);
          	_estimate *= up;
        }

	virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

};

}

}

#endif
