#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <vector>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>

using namespace std;
using namespace gtsam;

int main()
{

  //　向量保存好模拟的位姿和测量，到时候一个个往isam2里填加
  std::vector< BetweenFactor<Pose3> > gra;
  std::vector< Pose3 > initPose;

  // For simplicity, we will use the same noise model for odometry and loop closures
  noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Variances(
          (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());

  gra.push_back(BetweenFactor<Pose3>(1, 2, Pose3(Rot3::RzRyRx(0, 0, 0), Point3(5, 0, -1)), model));
  gra.push_back(BetweenFactor<Pose3>(2, 3, Pose3(Rot3::RzRyRx(0, 0, 0), Point3(5, 0, 2)), model));
  gra.push_back(BetweenFactor<Pose3>(3, 4, Pose3(Rot3::RzRyRx(0, 0, 0), Point3(0, 5, -1)), model));
  gra.push_back(BetweenFactor<Pose3>(4, 5, Pose3(Rot3::RzRyRx(0, 0, 0), Point3(-5, 0, -1)), model));
  gra.push_back(BetweenFactor<Pose3>(5, 2, Pose3(Rot3::RzRyRx(0, 0, 0), Point3(0, -5, 0)), model));

  initPose.push_back( Pose3(Rot3::RzRyRx(0, 0, 0), Point3(0.1, -0.1, 0)));
  initPose.push_back( Pose3(Rot3::RzRyRx(0, 0, 0), Point3(6, 0.1, -1.1)));
  initPose.push_back( Pose3(Rot3::RzRyRx(0, 0, 0), Point3(10.1, -0.1, 0.9)));
  initPose.push_back( Pose3(Rot3::RzRyRx(0, 0, 0), Point3(10, 5, 0)));
  initPose.push_back( Pose3(Rot3::RzRyRx(0, 0, 0), Point3(5.1, 4.9, -0.9)));

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  ISAM2 isam(parameters);

  NonlinearFactorGraph graph;
  Values initialEstimate;

  for( int i =0; i<5 ;i++)
  {
           initialEstimate.insert(i+1, initPose[i]);

           if(i == 0)
           {
               noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(
		(Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished()); 
               graph.push_back(PriorFactor<Pose3>(1, Pose3(Rot3::RzRyRx(0, 0, 0), Point3(0, 0, 0)), priorNoise));

           }else
           {

               graph.push_back(gra[i-1]);  // ie: when i = 1 , robot at pos2,  there is a edge gra[0] between pos1 and pos2
               if(i == 4)
               {
                   graph.push_back(gra[4]);  //  when robot at pos5, there two edge, one is pos4 ->pos5, another is pos5->pos2  (grad[4])
               }
               isam.update(graph, initialEstimate);
               isam.update();

               Values currentEstimate = isam.calculateEstimate();
               cout << "****************************************************" << endl;
               cout << "Frame " << i << ": " << endl;
               currentEstimate.print("Current estimate: ");

               // Clear the factor graph and values for the next iteration
               // 特别重要，update以后，清空原来的约束。已经加入到isam2的那些会用bayes tree保管，你不用操心了。
               graph.resize(0);
               initialEstimate.clear();
           }
  }
  return 0; 
}

