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

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

#include "frame.h"
#include "loop_closing.h"

using namespace std;
using namespace boost;
namespace fs  = boost::filesystem;

namespace slam
{

class LoopClosing;

class Graph
{

public:

	/** \brief Class constructor
   * \param Loop closing object pointer
   */
  Graph(LoopClosing* loop_closing);

  /** \brief Initialize the graph
   */
  bool init();

  /** \brief Starts graph
   */
  void run();

  /** \brief Add a frame to the queue of frames to be inserted into the graph as vertices
   * \param The frame to be inserted
   */
  void addFrameToQueue(Frame frame);

  /** \brief Find the non-consecutive closest neighbors of a given vertex
   * @return the list of neighbors
   * \param the vertex id
   */
  vector<int> findClosestNeighbors(int vertex_id);

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

  /** \brief Set the transformation between camera and robot odometry frame
   * \param the transform
   */
  inline void setCamera2Odom(const tf::Transform& camera2odom){camera2odom_ = camera2odom;}

  /** \brief Set camera matrix
   * \param camera matrix
   */
  inline void setCameraMatrix(const Mat& camera_matrix){camera_matrix_ = camera_matrix;}

  /** \brief Get camera matrix
   */
  inline Mat getCameraMatrix() const {return camera_matrix_;}

protected:

  /** \brief Check if there are frames in the queue to be inserted into the graph
   * @return true if frames queue is not empty.
   */
  bool checkNewFrameInQueue();

  /** \brief Converts the frame to a graph vertex and adds it to the graph
   */
  void processNewFrame();

  /** \brief Add a vertex to the graph
   * @return the vertex id
   * \param Vertex pose
   * \param Number of inliers between this vertex and the previous
   */
  int addVertex(tf::Transform pose,
                int inliers);

  /** \brief Save the graph to file
   */
  void saveToFile();

private:

  g2o::SparseOptimizer graph_optimizer_; //!> G2O graph optimizer

  list<Frame> frame_queue_; //!> Frames queue to be inserted into the graph

  mutex mutex_graph_; //!> Mutex for the graph manipulation

  mutex mutex_frame_queue_; //!> Mutex for the insertion of new frames into the graph

  tf::Transform camera2odom_; //!> Transformation between camera and robot odometry frame

  LoopClosing* loop_closing_; //!> Loop closing

  Mat camera_matrix_; //!> The camera matrix
};

} // namespace

#endif // GRAPH_H