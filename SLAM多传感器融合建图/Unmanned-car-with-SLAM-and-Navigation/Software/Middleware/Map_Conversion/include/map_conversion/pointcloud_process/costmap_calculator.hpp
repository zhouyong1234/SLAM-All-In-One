/*该类的功能主要是代价地图的计算
 edited by goldqiu -2022-04-9
*/
#ifndef MAP_CONVERSION_COSTMAP_CALCULATOR_HPP_
#define MAP_CONVERSION_COSTMAP_CALCULATOR_HPP_
#include "map_conversion/ros_topic_interface/cloud_data.hpp"
#include <yaml-cpp/yaml.h>
#include "map_conversion/utility.hpp"

namespace map_conversion {
class CostmapCalculator {
  public:
    CostmapCalculator(YAML::Node& config_node);
    CostmapCalculator() = default;
    
  private:

  private:

};
}

#endif