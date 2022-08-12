#include "Estimator.hpp"
int main(int argc,char** argv) {
  std::string configFile = argv[1];
  std::cout << "ConfigFile: " << configFile << std::endl; 
  vio::Estimator estimator(configFile);
  return 0;
}