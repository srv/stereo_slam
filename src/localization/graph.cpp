#include "localization/graph.h"
#include "common/tools.h"

using namespace tools;

namespace slam
{

  Graph::Graph()
  {
    init();
  }

  bool Graph::init()
  {
    // Initialize the g2o graph optimizer
    g2o::BlockSolverX::LinearSolverType * linear_solver_ptr;
    linear_solver_ptr = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linear_solver_ptr);
    g2o::OptimizationAlgorithmLevenberg * solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    graph_optimizer_.setAlgorithm(solver);
  }

  int Graph::addVertex(tf::Transform pose, int inliers)
  {
    // Convert pose for graph
    Eigen::Isometry3d vertex_pose = Tools::tfToIsometry(pose);

    // Set node id equal to graph size
    int id = graph_optimizer_.vertices().size();

    // Build the vertex
    slam::Vertex* cur_vertex = new slam::Vertex();
    cur_vertex->setId(id);
    cur_vertex->setEstimate(vertex_pose);
    if (id == 0)
    {
      // First time, no edges.
      cur_vertex->setFixed(true);
      graph_optimizer_.addVertex(cur_vertex);
    }
    else
    {
      // When graph has been initialized get the transform between current and previous vertices
      // and save it as an edge

      // Get last vertex
      slam::Vertex* prev_vertex = dynamic_cast<slam::Vertex*>(
        graph_optimizer_.vertices()[id - 1]);
      graph_optimizer_.addVertex(cur_vertex);

      double sigma = 1e+10;
      if (inliers > 0)
        sigma = (double)inliers;

      // Odometry edges
      g2o::EdgeSE3* e = new g2o::EdgeSE3();
      Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
      Eigen::Isometry3d t = prev_vertex->estimate().inverse() * cur_vertex->estimate();
      e->setVertex(0, prev_vertex);
      e->setVertex(1, cur_vertex);
      e->setMeasurement(t);
      e->setInformation(information/sigma);
      graph_optimizer_.addEdge(e);
    }

    return id;
  }

  void Graph::addEdge(int i, int j, tf::Transform edge, int inliers)
  {
    // Get the vertices
    slam::Vertex* v_i = dynamic_cast<slam::Vertex*>(graph_optimizer_.vertices()[i]);
    slam::Vertex* v_j = dynamic_cast<slam::Vertex*>(graph_optimizer_.vertices()[j]);

    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
    double sigma = 1e+10;
    if (inliers > 0)
      sigma = (double)inliers;

    // Add the new edge to graph
    g2o::EdgeSE3* e = new g2o::EdgeSE3();
    Eigen::Isometry3d t = Tools::tfToIsometry(edge.inverse());
    e->setVertex(0, v_i);
    e->setVertex(1, v_j);
    e->setMeasurement(t);
    e->setInformation(information/sigma);
    graph_optimizer_.addEdge(e);
  }

  void Graph::update()
  {
    graph_optimizer_.initializeOptimization();
    graph_optimizer_.optimize(20);
    ROS_INFO_STREAM("[Localization:] Optimization done in graph with " << graph_optimizer_.vertices().size() << " vertices.");
  }

} //namespace slam