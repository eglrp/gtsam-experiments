/*
* The GTSAM OdometryExample, modified to make use of symbols for keys.
* 
* 	- Robot poses are along the positive X-axis (horizontal, to the right in 3D)
* 	- The robot moves 2 meters forward each time step.
* 	- We have odometry measurements between poses
* 
*/


// We will use Pose2 variables to represent robot poses
#include <gtsam/geometry/Pose2.h>

// We will initialize the robot at the origin, using a PriorFactor
#include <gtsam/slam/PriorFactor.h>
// We will use a BetweenFactor to represent the transformation between successive poses
#include <gtsam/slam/BetweenFactor.h>

// We will use a nonlinear factor graph to represent the problem
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// We will use a Levenberg-Marquardt solver to optimize the factor graph
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been computed, we can calculate the marginal covariances
#include <gtsam/nonlinear/Marginals.h>

// Initial guesses and final values are stored in a Values container
#include <gtsam/nonlinear/Values.h>

// We will use symbols as a key for variables
#include <gtsam/inference/Symbol.h>


// Declaring namespaces
using namespace std;
using namespace gtsam;


// Main function
int main(int argc, char **argv){

	// Create an empty nonlinear factor graph
	NonlinearFactorGraph graph;

	// Add a prior factor on the first pose, setting it to the origin
	// A prior factor consists of a mean estimate, and a covariance model
	Pose2 priorMean(0.0, 0.0, 0.0);
	noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.3, 0.3, 0.1));
	// Note the use of a symbol here, as opposed to an integer key
	graph.add(PriorFactor<Pose2>(Symbol('X',0), priorMean, priorNoise));

	// Add odometry factors
	Pose2 odomMean(2.0, 0.0, 0.0);
	// For simplicity, we will use the same noise model for all odometry factors
	noiseModel::Diagonal::shared_ptr odomNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.1));
	// Create odometry (BetweenFactors) for consecutive poses
	graph.add(BetweenFactor<Pose2>(Symbol('X',0), Symbol('X',1), odomMean, odomNoise));
	graph.add(BetweenFactor<Pose2>(Symbol('X',1), Symbol('X',2), odomMean, odomNoise));
	graph.print("Factor graph: ");

	// Initialize the initial estimate
	// For illustrative purposes, we set these to incorrect values
	Values initial;
	initial.insert(Symbol('X',0), Pose2(0.5, 0.0, 0.2));
	initial.insert(Symbol('X',1), Pose2(2.3, 0.1, -0.2));
	initial.insert(Symbol('X',2), Pose2(4.1, 0.1, 0.1));
	initial.print("Initial estimate: ");

	// Optimize using a Levenberg-Marquardt solver
	Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
	result.print("Final result: ");

	// Calculate and print marginal covariances for all variables
	cout.precision(2);
	Marginals marginals(graph, result);
	cout << "X0: " << marginals.marginalCovariance(Symbol('X',0)) << endl;
	cout << "X1: " << marginals.marginalCovariance(Symbol('X',1)) << endl;
	cout << "X2: " << marginals.marginalCovariance(Symbol('X',2)) << endl;

	return 0;
}
