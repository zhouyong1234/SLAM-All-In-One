#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
using namespace std;
using namespace gtsam;
int main(int argc, char** argv) 
{
    NonlinearFactorGraph graph;

    Values initials;
    initials.insert(Symbol('x', 1), Pose3(Rot3::RzRyRx(0, 0, 0), Point3(0.1, -0.1, 0)));
    initials.insert(Symbol('x', 2), Pose3(Rot3::RzRyRx(0, 0, 0), Point3(6, 0.1, -1.1)));
    initials.insert(Symbol('x', 3), Pose3(Rot3::RzRyRx(0, 0, 0), Point3(10.1, -0.1, 0.9)));
    initials.insert(Symbol('x', 4), Pose3(Rot3::RzRyRx(0, 0, 0), Point3(10, 5, 0)));
    initials.insert(Symbol('x', 5), Pose3(Rot3::RzRyRx(0, 0, 0), Point3(5.1, 4.9, -0.9)));
    initials.print("\nInitial Values:\n"); 
    //固定第一个顶点，在gtsam中相当于添加一个先验因子
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(
          (Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished()); 
    graph.add(PriorFactor<Pose3>(Symbol('x', 1), Pose3(Rot3::RzRyRx(0, 0, 0), Point3(0, 0, 0)), priorNoise));

    //二元位姿因子
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances(
          (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    graph.add(BetweenFactor<Pose3>(Symbol('x', 1), Symbol('x', 2), Pose3(Rot3::RzRyRx(0, 0, 0), Point3(5, 0, -1)), odometryNoise));
    graph.add(BetweenFactor<Pose3>(Symbol('x', 2), Symbol('x', 3), Pose3(Rot3::RzRyRx(0, 0, 0), Point3(5, 0, 2)), odometryNoise));
    graph.add(BetweenFactor<Pose3>(Symbol('x', 3), Symbol('x', 4), Pose3(Rot3::RzRyRx(0, 0, 0), Point3(0, 5, -1)), odometryNoise));
    graph.add(BetweenFactor<Pose3>(Symbol('x', 4), Symbol('x', 5), Pose3(Rot3::RzRyRx(0, 0, 0), Point3(-5, 0, -1)), odometryNoise));

    //二元回环因子
    noiseModel::Diagonal::shared_ptr loopModel = noiseModel::Diagonal::Variances(
          (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    graph.add(BetweenFactor<Pose3>(Symbol('x', 5), Symbol('x', 2), Pose3(Rot3::RzRyRx(0, 0, 0), Point3(0, -5, 0)), loopModel));
//     graph.print("\nFactor Graph:\n"); 


    GaussNewtonParams parameters;
//     parameters.setVerbosity("ERROR");
    parameters.setMaxIterations(20);
    parameters.setLinearSolverType("MULTIFRONTAL_QR");
    GaussNewtonOptimizer optimizer(graph, initials, parameters);
    Values results = optimizer.optimize();
    results.print("Final Result:\n");

    Marginals marginals(graph, results);
//     cout << "x1 covariance:\n" << marginals.marginalCovariance(Symbol('x', 1)) << endl;
//     cout << "x2 covariance:\n" << marginals.marginalCovariance(Symbol('x', 2)) << endl;
//     cout << "x3 covariance:\n" << marginals.marginalCovariance(Symbol('x', 3)) << endl;
//     cout << "x4 covariance:\n" << marginals.marginalCovariance(Symbol('x', 4)) << endl;
//     cout << "x5 covariance:\n" << marginals.marginalCovariance(Symbol('x', 5)) << endl;

    return 0;
}

