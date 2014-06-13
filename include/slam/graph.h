/**
 * @file
 * @brief Graph class
 */

#ifndef GRAPH_H
#define GRAPH_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

using namespace std;

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

namespace stereo_slam
{

class Graph
{

public:

	// Constructor
  Graph();

  struct Params
  {
    int g2o_algorithm;                //!> Set to 0 for LinearSlam Solver with gauss-newton. Set to 1 for LinearSlam Solver with Levenberg.
    int go2_opt_max_iter;             //!> Maximum number of iteration for the graph optimization.
    string save_dir;                  //!> Directory where graph files will be saved.
    string pose_frame_id;             //!> Pose frame id for publisher.
    string pose_child_frame_id;       //!> Base frame id for publisher.

    // default settings
    Params () {
      g2o_algorithm               = 1;
      go2_opt_max_iter            = 20;
      save_dir                    = "";
      pose_frame_id               = "/map";
      pose_child_frame_id         = "/robot";
    }
  };

  /**
   * @param params new parameters
   */
  inline void setParams(const Params& params)
  {
    params_ = params;
    init();
  }

  /**
   * @return current parameters
   */
  inline Params params() const { return params_; }

  // Get the last pose of the graph (corrected graph pose and original odometry)
  void getLastPoses(tf::Transform current_odom, 
                    tf::Transform &last_graph_pose, 
                    tf::Transform &last_graph_odom);

  // Add a vertice to the graph
  int addVertice(tf::Transform current_odom, 
                 tf::Transform corrected_odom,
                 double timestamp);

  // Add an edge to the graph
  void addEdge(int i, int j, tf::Transform edge);

  // Optimize the graph
  void update();

  // Save the graph to file
  bool saveGraphToFile();

protected:

  bool init();

private:

	// Pose properties
  vector< pair<tf::Transform,double > > odom_history_;  //!> Vector to save the odometry history

  // G2O Optimization
  g2o::SparseOptimizer 
  	graph_optimizer_;								//!> G2O graph optimizer

  // Stores parameters
  Params params_;
};

} // namespace

#endif // GRAPH_H