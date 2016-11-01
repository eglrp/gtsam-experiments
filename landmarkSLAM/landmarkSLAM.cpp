/**
 * @file landmarkSLAM.cpp
 * @brief Simple SLAM example using odometry measurements and bearing-range (laser) measurements
 * @author J. Krishna Murthy
 */

/**
 * A simple 2D planar slam example with landmarks
 *  - Data from the robot's run (odometry and range-bearing observations) are present in example.graph
 *  - We use a BetweenFactor to represent an odometry observation
 *  - We use a BearingRangeFactor to represent a range-bearing observation
 */

// As this is a planar SLAM example, we will use Pose2 variables (x, y, theta) to represent
// the robot positions and Point2 variables (x, y) to represent the landmark coordinates.
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use a RangeBearing factor for the range-bearing measurements to identified
// landmarks, and Between factors for the relative motion described by odometry measurements.
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// common Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

// Utilities to read a .graph file and convert to a factor graph format
#include <gtsam/slam/dataset.h>


using namespace std;
using namespace gtsam;

int main(int argc, char **argv){

  // Input .graph filename
  string filename = "example.graph";

  // Construct a nonlinear factor graph from the .graph file
  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;
  SharedDiagonal model = noiseModel::Diagonal::Sigmas((Vector(3) << 0.05, 0.05, 2 * M_PI / 180.0));
  boost::tie(graph, initial) = load2D(filename, model);

  // Print the initial estimate
  // initial->print("Initial estimate: ");

  // Add a Gaussian prior on the first pose (assumed to be at the origin, along the positive X-axis)
  Pose2 priorMean(0.0, 0.0, 0.0);
  SharedDiagonal priorNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.01, 0.01, 0.01));
  graph->push_back(PriorFactor<Pose2>(0, priorMean, priorNoise));


  /*
   * Your code here
   *

   * In one version of the code, use the Gauss-Newton optimizer
   * In the second version of the code, use the Levenberg-Marquardt optimizer

   * NOTE: Do NOT create separate files for the Gauss-Newton and the Levenberg-Marquardt versions.
   *       Simply comment out one of the versions and keep the other uncommented.
   */

  

  return 0;
}

