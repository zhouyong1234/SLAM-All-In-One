#include "Simulator.hpp"
#include <random>
namespace vio{
Simulator::Simulator(const Config* cfg,const Camera* cam): 
  cfg_(cfg),
  cam_(cam){
}

void Simulator::createLandMarkers() {
  std::random_device sd;
  std::mt19937_64 genorator(sd());
  std::uniform_real_distribution<float> uniformDistX(-cfg_->simParam_.length[0]/2,cfg_->simParam_.length[0]/2);
  std::uniform_real_distribution<float> uniformDistY(-cfg_->simParam_.length[1]/2,cfg_->simParam_.length[1]/2);
  std::uniform_real_distribution<float> uniformDistZ(-cfg_->simParam_.length[2]/2,cfg_->simParam_.length[2]/2);
  landMarkers.clear();
  while (landMarkers.size() < cfg_->simParam_.featureSize) {
    float x = uniformDistX(genorator) + cfg_->simParam_.origin[0];
    float y = uniformDistY(genorator) + cfg_->simParam_.origin[1];
    float z = uniformDistZ(genorator) + cfg_->simParam_.origin[2];
    landMarkers.emplace_back(x,y,z);
  }
}
}