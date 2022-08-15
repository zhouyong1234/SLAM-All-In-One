#include <hdl_global_localization/bbs/bbs_localization.hpp>
#include <hdl_global_localization/bbs/occupancy_gridmap.hpp>

#include <queue>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>

namespace hdl_global_localization {

DiscreteTransformation::DiscreteTransformation() {
  this->level = -1;
}

DiscreteTransformation::DiscreteTransformation(int level, int x, int y, int theta) {
  this->level = level;
  this->x = x;
  this->y = y;
  this->theta = theta;
  this->score = 0.0;
}

DiscreteTransformation::~DiscreteTransformation() {}

bool DiscreteTransformation::operator<(const DiscreteTransformation& rhs) const {
  return score < rhs.score;
}

bool DiscreteTransformation::is_leaf() const {
  return level == 0;
}

Eigen::Isometry2f DiscreteTransformation::transformation(double theta_resolution, const std::vector<std::shared_ptr<OccupancyGridMap>>& gridmap_pyramid) {
  double trans_resolution = gridmap_pyramid[level]->grid_resolution();

  Eigen::Isometry2f trans = Eigen::Isometry2f::Identity();
  trans.linear() = Eigen::Rotation2Df(theta_resolution * theta).toRotationMatrix();
  trans.translation() = Eigen::Vector2f(trans_resolution * x, trans_resolution * y);

  return trans;
}

DiscreteTransformation::Points DiscreteTransformation::transform(const Points& points, double trans_resolution, double theta_resolution) {
  Points transformed(points.size());

  Eigen::Map<const Eigen::Matrix<float, 2, -1>> src(points[0].data(), 2, points.size());
  Eigen::Map<Eigen::Matrix<float, 2, -1>> dst(transformed[0].data(), 2, points.size());

  Eigen::Matrix2f rot = Eigen::Rotation2Df(theta_resolution * theta).toRotationMatrix();
  Eigen::Vector2f trans(trans_resolution * x, trans_resolution * y);

  dst = (rot * src).colwise() + trans;

  return transformed;
}

double DiscreteTransformation::calc_score(const Points& points, double theta_resolution, const std::vector<std::shared_ptr<OccupancyGridMap>>& gridmap_pyramid) {
  const auto& gridmap = gridmap_pyramid[level];
  auto transformed = transform(points, gridmap->grid_resolution(), theta_resolution);
  score = gridmap->calc_score(transformed);
  return score;
}

std::vector<DiscreteTransformation> DiscreteTransformation::branch() {
  std::vector<DiscreteTransformation> b;
  b.reserve(4);
  b.emplace_back(DiscreteTransformation(level - 1, x * 2, y * 2, theta));
  b.emplace_back(DiscreteTransformation(level - 1, x * 2 + 1, y * 2, theta));
  b.emplace_back(DiscreteTransformation(level - 1, x * 2, y * 2 + 1, theta));
  b.emplace_back(DiscreteTransformation(level - 1, x * 2 + 1, y * 2 + 1, theta));
  return b;
}

BBSLocalization::BBSLocalization(const BBSParams& params) : params(params) {}

BBSLocalization::~BBSLocalization() {}

void BBSLocalization::set_map(const BBSLocalization::Points& map_points, double resolution, int width, int height, int pyramid_levels, int max_points_per_cell) {
  gridmap_pyramid.resize(pyramid_levels);
  gridmap_pyramid[0].reset(new OccupancyGridMap(resolution, width, height));
  gridmap_pyramid[0]->insert_points(map_points, max_points_per_cell);

  for (int i = 1; i < pyramid_levels; i++) {
    gridmap_pyramid[i] = gridmap_pyramid[i - 1]->pyramid_up();
  }
}

boost::optional<Eigen::Isometry2f> BBSLocalization::localize(const BBSLocalization::Points& scan_points, double min_score, double* best_score) {
  theta_resolution = std::acos(1 - std::pow(gridmap_pyramid[0]->grid_resolution(), 2) / (2 * std::pow(params.max_range, 2)));

  double best_score_storage;
  best_score = best_score ? best_score : &best_score_storage;

  *best_score = min_score;
  boost::optional<DiscreteTransformation> best_trans;

  auto trans_queue = create_init_transset(scan_points);

  ROS_INFO_STREAM("Branch-and-Bound");
  while (!trans_queue.empty()) {
    // std::cout << trans_queue.size() << std::endl;

    auto trans = trans_queue.top();
    trans_queue.pop();

    if (trans.score < *best_score) {
      continue;
    }

    if (trans.is_leaf()) {
      best_trans = trans;
      *best_score = trans.score;
    } else {
      auto children = trans.branch();
      for (auto& child : children) {
        child.calc_score(scan_points, theta_resolution, gridmap_pyramid);
        trans_queue.push(child);
      }
    }
  }

  if (best_trans == boost::none) {
    return boost::none;
  }

  return best_trans->transformation(theta_resolution, gridmap_pyramid);
}

std::shared_ptr<const OccupancyGridMap> BBSLocalization::gridmap() const {
  return gridmap_pyramid[0];
}

std::priority_queue<DiscreteTransformation> BBSLocalization::create_init_transset(const Points& scan_points) const {
  double trans_res = gridmap_pyramid.back()->grid_resolution();
  std::pair<int, int> tx_range(std::floor(params.min_tx / trans_res), std::ceil(params.max_tx / trans_res));
  std::pair<int, int> ty_range(std::floor(params.min_ty / trans_res), std::ceil(params.max_ty / trans_res));
  std::pair<int, int> theta_range(std::floor(params.min_theta / theta_resolution), std::ceil(params.max_theta / theta_resolution));

  ROS_INFO_STREAM("Resolution trans:" << trans_res << " theta:" << theta_resolution);
  ROS_INFO_STREAM("TransX range:" << tx_range.first << " " << tx_range.second);
  ROS_INFO_STREAM("TransY range:" << ty_range.first << " " << ty_range.second);
  ROS_INFO_STREAM("Theta  range:" << theta_range.first << " " << theta_range.second);

  std::vector<DiscreteTransformation> transset;
  transset.reserve((tx_range.second - tx_range.first) * (ty_range.second - ty_range.first) * (theta_range.second - theta_range.first));
  for (int tx = tx_range.first; tx <= tx_range.second; tx++) {
    for (int ty = ty_range.first; ty <= ty_range.second; ty++) {
      for (int theta = theta_range.first; theta <= theta_range.second; theta++) {
        int level = gridmap_pyramid.size() - 1;
        transset.emplace_back(DiscreteTransformation(level, tx, ty, theta));
      }
    }
  }

  ROS_INFO_STREAM("Initial transformation set size:" << transset.size());

#pragma omp parallel for
  for (int i = 0; i < transset.size(); i++) {
    auto& trans = transset[i];
    const auto& gridmap = gridmap_pyramid[trans.level];
    trans.calc_score(scan_points, theta_resolution, gridmap_pyramid);
  }

  return std::priority_queue<DiscreteTransformation>(transset.begin(), transset.end());
}

}  // namespace  hdl_global_localization
