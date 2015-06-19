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
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

using namespace std;

namespace slam
{

class Graph
{

public:

	/** \brief Class constructor
   */
  Graph();

  /** \brief Initialize the grapth
   */
  bool init();

  /** \brief Add a vertex to the graph
   * \param Vertex pose
   * \param Number of inliers between this vertex and the previous
   */
  int addVertex(tf::Transform pose,
                int inliers);

  /** \brief Add an edge to the graph
   * \param Index of vertex 1
   * \param Index of vertex 2
   * \param Transformation between vertices
   * \param Number of inliers between these vertices
   */
  void addEdge(int i, int j, tf::Transform edge, int inliers);

  /** \brief Optimize the graph
   */
  void update();

private:

  g2o::SparseOptimizer graph_optimizer_; //!> G2O graph optimizer

};

} // namespace

#endif // GRAPH_H