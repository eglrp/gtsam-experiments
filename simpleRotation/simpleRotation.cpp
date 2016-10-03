/*

An adaptation of the Simple Rotation example provided by GTSAM.
Optimizes a single rotation according to a prior.

*/


// Our variable is a 2D rotation
#include <gtsam/geometry/Rot2.h>

// Each variable in the system (each pose) must be identified with a unique 'key'.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1)
// Here, we will use symbols
#include <gtsam/inference/Symbol.h>

// In GTSAM, measurement functions are represented as factors. Here, we will apply 
// a prior on the rotation, for which we will use 'PriorFactor'.
#include <gtsam/slam/PriorFactor.h>

// We will add all factors and form a factor graph. Since our factors are nonlinear, 
// we use a nonlinear factor graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// An initial guess for the value of a variable is held in a Values container
#include <gtsam/nonlinear/Values.h>

// We will use the Levenberg-Marquardt solver to solve the nonlinear system
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>


// Namespace declarations
using namespace std;
using namespace gtsam;


// Defining a constant to convert from degrees to radians
const double degree = M_PI / 180;


// Main function
int main(int argc, char **argv){

	// Create a factor to express a unary constraint.
	Rot2 prior = Rot2::fromAngle(30*degree);
	prior.print("True angle: ");
	// The noise model for the factor is an isotropic gaussian.
	noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(1, 1*degree);
	// Create a 'key' to index the factor by a variable
	Symbol key('x', 1);
	// Initialize the factor. The arguments passed are (key, value, noise model)
	PriorFactor<Rot2> factor(key, prior, model);

	// Create a graph container and add the factor to it
	NonlinearFactorGraph graph;
	graph.push_back(factor);
	graph.print("Factor graph: ");

	// Create an initial estimate
	Values initial;
	initial.insert(key, Rot2::fromAngle(20*degree));
	initial.print("Initial estimate: ");

	// Optimize using a Levenberg-Marquardt solver
	Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
	result.print("Final estimate: ");

	return 0;
}
