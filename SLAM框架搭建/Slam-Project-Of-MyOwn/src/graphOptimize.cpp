#include "graphOptimize.h"

namespace slam{

namespace optimizer{

GraphOptimize::GraphOptimize()
{

}

GraphOptimize::~GraphOptimize()
{

}

void GraphOptimize::createOptimizer()
{
	auto linearSolver = g2o::make_unique<SlamLinearSolver>();

	linearSolver->setBlockOrdering(false);

	g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( g2o::make_unique<SlamBlockSolver>( std::move(linearSolver) ) );

	optimizer.setAlgorithm(solver);
	
	optimizer.setVerbose(true);
}

void GraphOptimize::addVertex( const Eigen::Vector3f &pose, const int id )
{
	g2o::VertexSE2 *vertex = new g2o::VertexSE2();
	
	vertex->setId( id );

	vertex->setEstimate( g2o::SE2( pose(0), pose(1), pose(2) ) );

	optimizer.addVertex( vertex );

	std::cout<<"add a vertex to the optimizer ..."<<std::endl;
}

void GraphOptimize::addEdge( const Eigen::Vector3f &delta, 
			     const int from, 
                             const int to,  
                             Eigen::Matrix3d &information )
{
	edgeCount ++;

	g2o::EdgeSE2 *edge = new g2o::EdgeSE2();
	
	edge->vertices()[0] = optimizer.vertex( from );
	edge->vertices()[1] = optimizer.vertex( to );
	
	g2o::SE2 measurement( delta(0), delta(1), delta(2) );
	edge->setMeasurement( measurement );

	edge->setId( edgeCount );
	
	edge->setInformation( information );
	
	optimizer.addEdge( edge );

	//edgeCount ++;
	
	std::cout<<"add a edge to the optimize ... edge count: "<<edgeCount<<std::endl;
}

int GraphOptimize::execuateGraphOptimization()
{
	//slam::optimizer::VertexSE2* firstRobotPose = dynamic_cast<slam::optimizer::VertexSE2*>(optimizer.vertex(0));
        g2o::VertexSE2* firstRobotPose = dynamic_cast<g2o::VertexSE2*>(optimizer.vertex(0));
	
	firstRobotPose->setFixed(true);

	optimizer.initializeOptimization();
	
	int iter = 0;
	

	iter = optimizer.optimize( iteration );
	
	std::cout<<"execuate the graph optimization ... actual iteration: "<< iter << std::endl;
	
	optimizer.save("./result.g2o");

	return iteration; 
}

void GraphOptimize::getOptimizedResults()
{
	g2o::SparseOptimizer::VertexContainer nodes = optimizer.activeVertices();
	
	std::cout<<"------------------- Optimized Results -------------------"<<std::endl;
	std::cout<<"Estimated Nodes Size = "<<nodes.size()<<std::endl;
	std::cout<<"---------------------------------------------------------"<<std::endl;

	// TODO ....
	// for(g2o::SparseOptimizer::VertexContainer::const_iterator n = nodes.begin(); n != nodes.end(); ++n) {

	// 	//double estimate[3];
    //     	Eigen::Vector3d estimate;
	// 	if((*n)->getEstimateData(estimate)) {
    //     	//(*n)->getEstimateData(estimate);
	// 	//	std::cout << "result:" << estimate<< std::endl; 
 			
	// 		estimatedPoses.push_back( estimate.cast<float>()  );
	//        	}
    // 	}
	
}

void GraphOptimize::saveG2OFile( const std::string &filePath )
{
	optimizer.save( filePath.c_str() );
}

void GraphOptimize::setMaxIeration( const int iteration )
{
	this->iteration = iteration;
}

}

}

