#pragma once 

#include <Camera/Camera.h>
#include <vector>

namespace TGK {
namespace Geometry {

class Triangulator {
public:
    struct Config {
        double max_proj_res = 3.;
        double min_dist = 0.1;
        double max_dist = 50.;
    };

    Triangulator(const Config& config, const Camera::CameraPtr camera);

    bool Triangulate(const std::vector<Eigen::Matrix3d>& G_R_Cs, 
                     const std::vector<Eigen::Vector3d>& G_p_Cs,
                     const std::vector<Eigen::Vector2d>& im_pts,
                     Eigen::Vector3d* G_p);

private:
    Camera::CameraPtr camera_;
    Config config_;
};

} // namespace Geometry
}  // namespace TGK