#ifndef __VERTEX_POINT_XY_H_
#define __VERTEX_POINT_XY_H_

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"

#include <Eigen/Core>

namespace slam{

namespace optimizer{

class VertexPointXY : public g2o::BaseVertex<2, Eigen::Vector2f>
{

public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
 
        VertexPointXY();

        virtual void setToOriginImpl() 
	{
        	_estimate.setZero();
        }

        virtual void oplusImpl(const double* update)
        {
          	_estimate[0] += update[0];
          	_estimate[1] += update[1];
        }

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;


};

}

}


#endif
