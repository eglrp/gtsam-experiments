/*
* Adapted from the Pose2SLAMExample_graph example provided with GTSAM.
* 
* 	- Reads a .graph file (GTSAM format) and constructs a pose graph
* 	- Optimizes the pose graphs
* 	- Saves the graph in graphviz format for visualization (can be converted to PDF as well)
*/


#include <gtsam/slam/dataset.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/geometry/Pose2.h>
#include <fstream>


using namespace std;
using namespace gtsam;


int main(int argc, char **argv){

	// Read in file and build graph
	NonlinearFactorGraph::shared_ptr graph;
	Values::shared_ptr initial;
	SharedDiagonal model = noiseModel::Diagonal::Sigmas((Vector(3) << 0.05, 0.05, 5.0*M_PI/180.0));
	// string graphFile = "/home/km/libs/gtsam-3.2.1/examples/Data/w100.graph";
	// string graphFile = "/home/km/code/CAIR_Backend/multi_session_september_2016_data/graphFile.graph";
	// string graphFile = "/home/km/code/CAIR_Backend/multi_session_september_2016_data/sept_30_junaid/loop1/traj1_loop.g2o";
	string graphFile = "traj1_traj2_loop.g2o";
	boost::tie(graph, initial) = readG2o(graphFile, 0);
	initial->print("Initial estimate: ");

	// Add a Gaussian prior on the first pose
	// Pose2 priorMean(0.0, 0.0, 0.0);
	// SharedDiagonal priorSigma = noiseModel::Diagonal::Sigmas((Vector(3) << 0.01, 0.01, 0.01));
	// graph->push_back(PriorFactor<Pose2>(0, priorMean, priorSigma));

	// Perform single step optimization using Levenberg-Marquardt
	Values result = LevenbergMarquardtOptimizer(*graph, *initial).optimize();
	result.print("Result: ");

	// // Save the initial graph
	// ofstream inputGraph("inputGraph.dot");
	// graph->saveGraph(inputGraph, *initial);

	// // Save the final graph
	// ofstream outputGraph("outputGraph.dot");
	// graph->saveGraph(outputGraph, result);

	// Write g2o file
	string outputGraph = "outputGraph.g2o";
	writeG2o(*graph, result, outputGraph);

	return 0;
}
