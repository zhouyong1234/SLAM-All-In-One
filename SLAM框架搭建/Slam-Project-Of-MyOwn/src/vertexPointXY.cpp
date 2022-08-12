#include "vertexPointXY.h"

namespace slam{

namespace optimizer{

VertexPointXY::VertexPointXY() : g2o::BaseVertex<2, Eigen::Vector2f>()
{
	_estimate.setZero();
}

bool VertexPointXY::read(std::istream& is)
{
      is >> _estimate[0] >> _estimate[1];

      return true;
}

bool VertexPointXY::write(std::ostream& os) const
{
      os << estimate()(0) << " " << estimate()(1);

      return os.good();
}


}

}
