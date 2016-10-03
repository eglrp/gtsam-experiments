/*
* OdometryExample provided with GTSAM
*
*	- Robot poses are along the positive X-axis (horizontal, to the right in 2D)
* 	- The robot moves 2 meters forward each step.
* 	- We have odometry measurements between poses
*
*/


// We will use Pose2 variables to represent robot positions
#include <gtsam/geometry/Pose2.h>

// We initialize the robot at the origin, using a prior factor
#include <gtsam/slam/PriorFactor.h>
// We will use 'BetweenFactor' to describe relative pose measurements (odometry, here)
#include <gtsam/slam/BetweenFactor.h>

// We will use a factor graph to encapsulate all the factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// We will use the Levenberg-Marquardt solver to optimize the factor graph
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been computed, we can also calculate the marginal covariances
#include <gtsam/nonlinear/Marginals.h>

// Initial guesses and final values for variables are stored in a 'Values' container
#include <gtsam/nonlinear/Values.h>


// Declaring namespaces
using namespace std;
using namespace gtsam;


// Main function
int main(int argc, char **argv){

	// Create an empty nonlinear factor graph
	NonlinearFactorGraph graph;

	// Add a prior factor on the first pose, setting it to the origin
	// A prior factor consists of a mean and a covariance model
	Pose2 priorMean(0.0, 0.0, 0.0);
	noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.3, 0.3, 0.1));
	graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));

	// Add odometry factors
	Pose2 odomMean(2.0, 0.0, 0.0);
	// For simplicity, we will use the same noise model for each of the odometry factors
	noiseModel::Diagonal::shared_ptr odomNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.1));
	// Create odometry (BetweenFactors) for consecutive poses
	graph.add(BetweenFactor<Pose2>(1, 2, odomMean, odomNoise));
	graph.add(BetweenFactor<Pose2>(2, 3, odomMean, odomNoise));
	graph.print("Factor graph: ");

	// Initialize the initial estimate
	// For illustrative purposes, we set these to incorrect values
	Values initial;
	initial.insert(1, Pose2(0.5, 0.0, 0.2));
	initial.insert(2, Pose2(2.3, 0.1, -0.2));
	initial.insert(3, Pose2(4.1, 0.1, 0.1));
	initial.print("Initial estimate: ");

	// Optimize using a Levenberg-Marquardt solver
	Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
	result.print("Final result: ");

	// Calculate and print marginal covariances for all variables
	cout.precision(2);
	Marginals marginals(graph, result);
	cout << "X1: " << marginals.marginalCovariance(1) << endl;
	cout << "X2: " << marginals.marginalCovariance(2) << endl;
	cout << "X3: " << marginals.marginalCovariance(3) << endl;

	return 0;
}
