#include "graph.h"
#include "tools.h"

/** \brief Class constructor. Reads node parameters and initialize some properties.
  * @return 
  */
stereo_slam::Graph::Graph()
{
  init();
}

/** \brief Init the graph
  * @return 
  */
bool stereo_slam::Graph::init()
{
  // Initialize the g2o graph optimizer
    if (params_.g2o_algorithm == 0)
    {
      // Slam linear solver with gauss-newton
      SlamLinearSolver* linear_solver_ptr = new SlamLinearSolver();
      linear_solver_ptr->setBlockOrdering(false);
      SlamBlockSolver* block_solver_ptr = new SlamBlockSolver(linear_solver_ptr);
      g2o::OptimizationAlgorithmGaussNewton* solver_gauss_ptr = 
        new g2o::OptimizationAlgorithmGaussNewton(block_solver_ptr);
      graph_optimizer_.setAlgorithm(solver_gauss_ptr);
    }
    else if (params_.g2o_algorithm == 1)
    {
      // Linear solver with Levenberg
      g2o::BlockSolverX::LinearSolverType * linear_solver_ptr;
      linear_solver_ptr = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();
      g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linear_solver_ptr);
      g2o::OptimizationAlgorithmLevenberg * solver = 
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
      graph_optimizer_.setAlgorithm(solver);
    }
    else
    {
      ROS_ERROR("[StereoSlam:] g2o_algorithm parameter must be 0 or 1.");
      return false;
    }  
}

/** \brief Get last poses of the graph
  * @return 
  * \param Current odometry pose used in the case of graph is empty.
  * \param Contains the last graph pose.
  * \param Contains the last odom pose.
  */
void stereo_slam::Graph::getLastPoses(tf::Transform current_odom, 
                                      tf::Transform &last_graph_pose, 
                                      tf::Transform &last_graph_odom)
{
  // Init
  last_graph_pose = current_odom;
  last_graph_odom = current_odom;

  // Get last
  int last_idx = graph_optimizer_.vertices().size() - 1;
  if (odom_history_.size() > 0 && last_idx >= 0)
  {
    // Get the last optimized pose
    g2o::VertexSE3* last_vertex =  dynamic_cast<g2o::VertexSE3*>
          (graph_optimizer_.vertices()[last_idx]);
    last_graph_pose = stereo_slam::Tools::getVertexPose(last_vertex);

    // Original odometry
    last_graph_odom = odom_history_.at(last_idx).first;
  }

  return;
}

/** \brief Add new vertice into the graph
  * @return 
  * \param Last corrected odometry pose.
  * \param Current readed odometry.
  * \param Timestamp for the current odometry.
  */
int stereo_slam::Graph::addVertice(tf::Transform current_odom, 
                                   tf::Transform corrected_odom,
                                   double timestamp)
{
  // Convert pose for graph
  Eigen::Isometry3d vertice_pose = stereo_slam::Tools::tfToEigen(corrected_odom);

  // Set node id equal to graph size
  int id = graph_optimizer_.vertices().size();

  // Build the vertex
  g2o::VertexSE3* cur_vertex = new g2o::VertexSE3();
  cur_vertex->setId(id);
  cur_vertex->setEstimate(vertice_pose);
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
    g2o::VertexSE3* prev_vertex = dynamic_cast<g2o::VertexSE3*>(
      graph_optimizer_.vertices()[id - 1]);
    graph_optimizer_.addVertex(cur_vertex);

    // Odometry edges
    g2o::EdgeSE3* e = new g2o::EdgeSE3();
    Eigen::Isometry3d t = prev_vertex->estimate().inverse() * cur_vertex->estimate();
    e->setVertex(0, prev_vertex);
    e->setVertex(1, cur_vertex);
    e->setMeasurement(t);
    graph_optimizer_.addEdge(e);
  }

  // Save the original odometry for this new node
  odom_history_.push_back(make_pair(current_odom, timestamp));

  return id;
}

/** \brief Add new edge to the graph
  * @return 
  * \param Id vertice 1.
  * \param Id vertice 2.
  * \param Edge transform that joins both vertices.
  */
void stereo_slam::Graph::addEdge(int i, int j, tf::Transform edge)
{
  // TODO: Check size of graph

  // Get the vertices
  g2o::VertexSE3* v_i = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[i]);
  g2o::VertexSE3* v_j = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[j]);

  // Add the new edge to graph
  g2o::EdgeSE3* e = new g2o::EdgeSE3();
  Eigen::Isometry3d t = stereo_slam::Tools::tfToEigen(edge);
  e->setVertex(0, v_j);
  e->setVertex(1, v_i);
  e->setMeasurement(t);
  graph_optimizer_.addEdge(e);
}

/** \brief Update the graph
  * @return 
  */
void stereo_slam::Graph::update()
{
    graph_optimizer_.initializeOptimization();
    graph_optimizer_.optimize(params_.go2_opt_max_iter);
    ROS_INFO_STREAM("[StereoSlam:] Optimization done in graph with " << graph_optimizer_.vertices().size() << " vertices.");
}

/** \brief Save the optimized graph into a file with the same format than odometry_msgs.
  * @return
  */
bool stereo_slam::Graph::saveGraphToFile()
{
  string block_file, vertices_file, edges_file;
  vertices_file = params_.save_dir + "graph_vertices.txt";
  edges_file = params_.save_dir + "graph_edges.txt";
  block_file = params_.save_dir + ".graph.block";

  // Create a blocking element
  fstream f_block(block_file.c_str(), ios::out | ios::trunc);

  // Open to append
  fstream f_vertices(vertices_file.c_str(), ios::out | ios::trunc);
  fstream f_edges(edges_file.c_str(), ios::out | ios::trunc);
  
  // Output the vertices file
  for (unsigned int i=0; i<graph_optimizer_.vertices().size(); i++)
  {
    // Get timestamp
    double timestamp = odom_history_.at(i).second;

    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[i]);
    tf::Transform pose = stereo_slam::Tools::getVertexPose(v);
    f_vertices <<  setprecision(19) << 
          timestamp  << "," << 
          i << "," << 
          timestamp << "," << 
          params_.pose_frame_id << "," << 
          params_.pose_child_frame_id << "," << 
          setprecision(6) << 
          pose.getOrigin().x() << "," << 
          pose.getOrigin().y() << "," << 
          pose.getOrigin().z() << "," << 
          pose.getRotation().x() << "," << 
          pose.getRotation().y() << "," << 
          pose.getRotation().z() << "," << 
          pose.getRotation().w() <<  endl;
  }
  f_vertices.close();

  // Output the edges file
  int counter = 0;
  for ( g2o::OptimizableGraph::EdgeSet::iterator it=graph_optimizer_.edges().begin();
        it!=graph_optimizer_.edges().end(); it++)
  {
    g2o::EdgeSE3* e = dynamic_cast<g2o::EdgeSE3*> (*it);
    if (e)
    {
      // Only take into account non-directed edges
      if (abs(e->vertices()[0]->id() - e->vertices()[1]->id()) > 1 )
      {
        g2o::VertexSE3* v_0 = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[e->vertices()[0]->id()]);
        g2o::VertexSE3* v_1 = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[e->vertices()[1]->id()]);
        tf::Transform pose_0 = stereo_slam::Tools::getVertexPose(v_0);
        tf::Transform pose_1 = stereo_slam::Tools::getVertexPose(v_1);

        f_edges << counter << "," << 
              setprecision(6) << 
              pose_0.getOrigin().x() << "," << 
              pose_0.getOrigin().y() << "," << 
              pose_0.getOrigin().z() << "," << 
              pose_1.getOrigin().x() << "," << 
              pose_1.getOrigin().y() << "," << 
              pose_1.getOrigin().z() <<  endl;
        counter++;
      }
    }
  }
  f_edges.close();

  // Un-block
  f_block.close();
  int ret_code = remove(block_file.c_str());
  if (ret_code != 0)
  {
    ROS_ERROR("[StereoSlam:] Error deleting the blocking file.");   
    return false;
  }

  return true;
}

