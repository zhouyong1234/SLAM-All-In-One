#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <vector>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>

using namespace std;
using namespace gtsam;

int main()
{

  //　向量保存好模拟的位姿和测量，到时候一个个往isam2里填加
  std::vector< BetweenFactor<Pose2> > gra;
  std::vector< Pose2 > initPose;

  noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

  gra.push_back(BetweenFactor<Pose2>(1, 2, Pose2(2, 0, 0     ), model));
  gra.push_back(BetweenFactor<Pose2>(2, 3, Pose2(2, 0, M_PI_2), model));
  gra.push_back(BetweenFactor<Pose2>(3, 4, Pose2(2, 0, M_PI_2), model));
  gra.push_back(BetweenFactor<Pose2>(4, 5, Pose2(2, 0, M_PI_2), model));
  gra.push_back(BetweenFactor<Pose2>(5, 2, Pose2(2, 0, M_PI_2), model));

  initPose.push_back(Pose2(0.5, 0.0,  0.2   ));
  initPose.push_back( Pose2(2.3, 0.1, -0.2   ));
  initPose.push_back( Pose2(4.1, 0.1,  M_PI_2));
  initPose.push_back( Pose2(4.0, 2.0,  M_PI  ));
  initPose.push_back( Pose2(2.1, 2.1, -M_PI_2));

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  ISAM2 isam(parameters);

  // 注意isam2的graph里只添加isam2更新状态以后新测量到的约束
  NonlinearFactorGraph graph;
  Values initialEstimate;

  // the first pose don't need to update
  for( int i =0; i<5 ;i++)
  {
          // Add an initial guess for the current pose
           initialEstimate.insert(i+1, initPose[i]);

           if(i == 0)
           {
               //  Add a prior on the first pose, setting it to the origin
               // A prior factor consists of a mean and a noise model (covariance matrix)
               noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
               graph.push_back(PriorFactor<Pose2>(1, Pose2(0, 0, 0), priorNoise));

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

