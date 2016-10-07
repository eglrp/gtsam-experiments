/*
* Localization example, provided by GTSAM.
* 
* A simple pose SLAM example with "GPS-like" measurements
*
* 	- The robot moves forward 2 meters, each time step
* 	- The robot initially faces the positive X-axis
* 	- We have odometry information between poses
* 	- We have "GPS-like" measurements implemented with a custom factor
*
*/


// We will use Pose2 variables (x, y, theta) to represent the robot pose
#include <gtsam/geometry/Pose2.h>

// We will use simple integer keys to refer to the variables
#include <gtsam/inference/Key.h>

// We use between factors to model odometry measurements
#include <gtsam/slam/BetweenFactor.h>

// We use a nonlinear factor graph to represent our problem
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// We use the Values container to store initial values and results
#include <gtsam/nonlinear/Values.h>

// We use a Levenberg-Marquardt solver to solve the optimization problem
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// We are also interested in the marginal covariances
#include <gtsam/nonlinear/Marginals.h>

// We want to define our own (nonlinear) factor
#include <gtsam/nonlinear/NonlinearFactor.h>


// Declaring namespaces
using namespace std;
using namespace gtsam;


// We define a custom unary factor to implement a "GPS-like" functionality
// Standard GPS measurements provide information only on the position, not orientation
// The factor will be a unary factor, i.e., it will affect only a single system variable
// We define our "UnaryFactor" by extending the "NoiseModelFactor1" class
class UnaryFactor : public NoiseModelFactor1<Pose2> {

	// The factor will hold a measurement consisting of an (x, y) location
	Point2 pos_;

public:

	// Shorthand for a smart pointer to a factor
	typedef boost::shared_ptr<UnaryFactor> shared_ptr;

	// The constructor requires the variable key, the (x, y) measurement value, and the noise model
	UnaryFactor(Key j, Point2 pos, const SharedNoiseModel& model) : 
		NoiseModelFactor1<Pose2>(model, j), pos_(pos) {}

	// Destructor
	virtual ~UnaryFactor() {}

	// Using the NoiseModelFactor1 class, there are two functions that must be overridden.
	// The first is the 'evaluateError' function. This function implements the desired measurement 
	// function, returning a vector of errors when evaluated at the provided variable value. It must 
	// also calculate the Jacobians for this measurement function, if requested.
	Vector evaluateError(const Pose2& q, boost::optional<Matrix&> H = boost::none) const {
		// The measurement function for a GPS-like measurement is simple:
		//		error.x = pose.x - measurement.x
		// 		error.y = pose.y - measurement.y
		// Consequently, the Jacobians are:
		// 		[derror.x/dx  derror.x/dy  derror.x/dtheta] = [1 0 0]
		// 		[derror.y/dx  derror.y/dy  derror.y/dtheta] = [0 1 0]
		if(H){
			(*H) = (Matrix(2,3) << 1.0, 0.0,0.0, 0.0, 1.0, 0.0);
		}
		return (Vector(2) << q.x() - pos_.x(), q.y() - pos_.y());
	}

	// The second is a clone function that allows the factor to be copied. Under most circumstances, 
	// the following constructor that employs the default copy constructor should work fine.
	virtual gtsam::NonlinearFactor::shared_ptr clone() const {
		return boost::static_pointer_cast<gtsam::NonlinearFactor>(
			gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this)));
	}

};


// Main function
int main(int argc, char **argv){

	// Create a nonlinear factor graph
	NonlinearFactorGraph graph;

	// Add odometry factors
	noiseModel::Diagonal::shared_ptr odomNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.1));
	graph.add(BetweenFactor<Pose2>(1, 2, Pose2(2.0, 0.0, 0.0), odomNoise));
	graph.add(BetweenFactor<Pose2>(2, 3, Pose2(2.0, 0.0, 0.0), odomNoise));

	// Add "GPS-like" measurements
	noiseModel::Diagonal::shared_ptr unaryNoise = noiseModel::Diagonal::Sigmas((Vector(2) << 0.1, 0.1));
	graph.add(boost::make_shared<UnaryFactor>(1, Point2(0.0, 0.0), unaryNoise));
	graph.add(boost::make_shared<UnaryFactor>(2, Point2(2.0, 0.0), unaryNoise));
	graph.add(boost::make_shared<UnaryFactor>(3, Point2(4.0, 0.0), unaryNoise));
	graph.print("Factor graph: ");

	// Provide an initial estimate
	Values initialEstimate;
	initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
	initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2));
	initialEstimate.insert(3, Pose2(4.1, 0.1, 0.1));
	initialEstimate.print("Initial Estimate: ");

	// Optimize
	LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
	Values result = optimizer.optimize();
	result.print("Final Estimate: ");

	// Get marginal covariances
	Marginals marginals(graph, result);
	cout << "X1: " << marginals.marginalCovariance(1) << endl;
	cout << "X2: " << marginals.marginalCovariance(2) << endl;
	cout << "X3: " << marginals.marginalCovariance(3) << endl;

	return 0;
}
