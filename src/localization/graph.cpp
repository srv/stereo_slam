#include <string>
#include <fstream>
#include <streambuf>
#include <boost/filesystem.hpp>
#include "localization/graph.h"
#include "localization/vertex.h"

using namespace boost;
namespace fs=filesystem;

/** \brief Class constructor. Reads node parameters and initialize some properties.
  * @return
  */
slam::Graph::Graph()
{
  init();
}

/** \brief Init the graph
  * @return
  */
bool slam::Graph::init()
{
  // Delete all the files (if any)
  string block_file, vertices_file, edges_file;
  vertices_file = params_.save_dir + "graph_vertices.txt";
  edges_file = params_.save_dir + "graph_edges.txt";
  block_file = params_.save_dir + ".graph.lock";
  fs::remove(vertices_file);
  fs::remove(edges_file);
  fs::remove(block_file);

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
    ROS_ERROR("[Localization:] g2o_algorithm parameter must be 0 or 1.");
    return false;
  }
}

/** \brief Get last vertex id
  * @return
  */
int slam::Graph::getLastVertexId()
{
  return graph_optimizer_.vertices().size() - 1;
}

/** \brief Get last poses of the graph
  * @return
  * \param Current odometry pose used in the case of graph is empty.
  * \param Contains the last graph pose.
  * \param Contains the last odometry pose.
  */
void slam::Graph::getLastPoses(tf::Transform current_odom,
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
    slam::Vertex* last_vertex =  dynamic_cast<slam::Vertex*>
          (graph_optimizer_.vertices()[last_idx]);
    last_graph_pose = Tools::getVertexPose(last_vertex);

    // Original odometry
    last_graph_odom = odom_history_.at(last_idx).first;
  }

  return;
}

/** \brief Get the best neighbors by distance
  * @return
  * \param Number of previous candidates to be discarted
  * \param Number of neighbors to be retrieved.
  * \param Will contain the list of best neighbors by distance.
  */
void slam::Graph::findClosestNodes(int discart_first_n, int best_n, vector<int> &neighbors)
{
  // Init
  neighbors.clear();

  // Get the pose of last graph node
  int last_idx = graph_optimizer_.vertices().size() - 1;
  if (last_idx < 0) return;
  slam::Vertex* last_vertex =  dynamic_cast<slam::Vertex*>
        (graph_optimizer_.vertices()[last_idx]);
  tf::Transform last_graph_pose = Tools::getVertexPose(last_vertex);

  // Loop thought all the other nodes
  vector< pair< int,double > > neighbor_distances;
  for (int i=last_idx-discart_first_n-1; i>=0; i--)
  {
    // Get the node pose
    slam::Vertex* cur_vertex =  dynamic_cast<slam::Vertex*>
            (graph_optimizer_.vertices()[i]);
    tf::Transform cur_pose = Tools::getVertexPose(cur_vertex);
    double dist = Tools::poseDiff(cur_pose, last_graph_pose);
    neighbor_distances.push_back(make_pair(i, dist));
  }

  // Exit if no neighbors
  if (neighbor_distances.size() == 0) return;

  // Sort the neighbors
  sort(neighbor_distances.begin(), neighbor_distances.end(), Tools::sortByDistance);

  // First
  neighbors.push_back(neighbor_distances[0].first);

  // Min number
  if (neighbor_distances.size() < best_n)
    best_n = neighbor_distances.size();

  // Get the best non-consecutive n nodes
  for (uint i=1; i<neighbor_distances.size(); i++)
  {
    if (abs(neighbor_distances[i-1].first - neighbor_distances[i].first) > 3)
      neighbors.push_back(neighbor_distances[i].first);
    if (neighbors.size() == best_n)
      break;
  }
}

/** \brief Add new vertex into the graph
  * @return
  * \param Last corrected odometry pose.
  * \param Current read odometry.
  * \param Timestamp for the current odometry.
  */
int slam::Graph::addVertex(tf::Transform pose)
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

    // Odometry edges
    g2o::EdgeSE3* e = new g2o::EdgeSE3();
    Eigen::Isometry3d t = prev_vertex->estimate().inverse() * cur_vertex->estimate();
    e->setVertex(0, prev_vertex);
    e->setVertex(1, cur_vertex);
    e->setMeasurement(t);
    graph_optimizer_.addEdge(e);
  }

  return id;
}

/** \brief Add new vertex into the graph
  * @return
  * \param Last estimated pose.
  * \param Last corrected pose.
  * \param Timestamp for the current odometry.
  */
int slam::Graph::addVertex(tf::Transform pose,
                            tf::Transform pose_corrected,
                            double timestamp)
{
  // Save the original odometry for this new node
  odom_history_.push_back(make_pair(pose, timestamp));

  // Add the node
  return addVertex(pose_corrected);
}

/** \brief Add new edge to the graph
  * @return
  * \param Id vertex 1.
  * \param Id vertex 2.
  * \param Edge transform that joins both vertices.
  */
void slam::Graph::addEdge(int i, int j, tf::Transform edge)
{
  // TODO: Check size of graph

  // Get the vertices
  slam::Vertex* v_i = dynamic_cast<slam::Vertex*>(graph_optimizer_.vertices()[i]);
  slam::Vertex* v_j = dynamic_cast<slam::Vertex*>(graph_optimizer_.vertices()[j]);

  // Add the new edge to graph
  g2o::EdgeSE3* e = new g2o::EdgeSE3();
  Eigen::Isometry3d t = Tools::tfToIsometry(edge);
  e->setVertex(0, v_j);
  e->setVertex(1, v_i);
  e->setMeasurement(t);
  graph_optimizer_.addEdge(e);
}

/** \brief Updates vertex estimate
  * @return
  * \param Id vertex.
  * \param New estimate.
  */
void slam::Graph::setVertexEstimate(int vertex_id, tf::Transform pose)
{
  dynamic_cast<slam::Vertex*>(graph_optimizer_.vertices()[vertex_id])->setEstimate(Tools::tfToIsometry(pose));
}

/** \brief Update the graph
  */
void slam::Graph::update()
{
    graph_optimizer_.initializeOptimization();
    graph_optimizer_.optimize(params_.go2_opt_max_iter);
    ROS_INFO_STREAM("[Localization:] Optimization done in graph with " << graph_optimizer_.vertices().size() << " vertices.");
}

/** \brief Save the optimized graph into a file with the same format than odometry_msgs.
  * @return
  */
bool slam::Graph::saveToFile()
{
  string block_file, vertices_file, edges_file;
  vertices_file = params_.save_dir + "graph_vertices.txt";
  edges_file = params_.save_dir + "graph_edges.txt";
  block_file = params_.save_dir + ".graph.lock";

  // Wait until lock file has been released
  while(fs::exists(block_file)){}

  // Create a locking element
  fstream f_block(block_file.c_str(), ios::out | ios::trunc);

  // Open to append
  fstream f_vertices(vertices_file.c_str(), ios::out | ios::trunc);
  fstream f_edges(edges_file.c_str(), ios::out | ios::trunc);

  // Output the vertices file
  for (unsigned int i=0; i<graph_optimizer_.vertices().size(); i++)
  {
    // Get timestamp
    double timestamp = odom_history_.at(i).second;

    slam::Vertex* v = dynamic_cast<slam::Vertex*>(graph_optimizer_.vertices()[i]);
    tf::Transform pose = Tools::getVertexPose(v);
    f_vertices << fixed << setprecision(9) <<
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
        slam::Vertex* v_0 = dynamic_cast<slam::Vertex*>(graph_optimizer_.vertices()[e->vertices()[0]->id()]);
        slam::Vertex* v_1 = dynamic_cast<slam::Vertex*>(graph_optimizer_.vertices()[e->vertices()[1]->id()]);
        tf::Transform pose_0 = Tools::getVertexPose(v_0);
        tf::Transform pose_1 = Tools::getVertexPose(v_1);

        f_edges <<
              e->vertices()[0]->id() << "," <<
              e->vertices()[1]->id() << "," <<
              setprecision(6) <<
              pose_0.getOrigin().x() << "," <<
              pose_0.getOrigin().y() << "," <<
              pose_0.getOrigin().z() << "," <<
              pose_0.getRotation().x() << "," <<
              pose_0.getRotation().y() << "," <<
              pose_0.getRotation().z() << "," <<
              pose_0.getRotation().w() << "," <<
              pose_1.getOrigin().x() << "," <<
              pose_1.getOrigin().y() << "," <<
              pose_1.getOrigin().z() << "," <<
              pose_1.getRotation().x() << "," <<
              pose_1.getRotation().y() << "," <<
              pose_1.getRotation().z() << "," <<
              pose_1.getRotation().w() << endl;
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
    ROS_ERROR("[Localization:] Error deleting the blocking file.");
    return false;
  }

  return true;
}


/** \brief Reads the graph vertices file and return one string with all the contents
  * @return
  */
string slam::Graph::readFile()
{
  string block_file, vertices_file, output;
  vertices_file = params_.save_dir + "graph_vertices.txt";
  block_file = params_.save_dir + ".graph.lock";

  // Check if file exists
  if (!fs::exists(vertices_file))
  {
    ROS_WARN("[Localization:] The graph vertices file does not exists.");
    return output;
  }

  // Wait until lock file has been released
  while(fs::exists(block_file)){}

  // Create a locking element
  fstream f_block(block_file.c_str(), ios::out | ios::trunc);

  // Read the graph vertices
  ifstream vertices(vertices_file.c_str());
  string file((istreambuf_iterator<char>(vertices)),
               istreambuf_iterator<char>());
  output = file;

  // Un-block
  f_block.close();
  int ret_code = remove(block_file.c_str());
  if (ret_code != 0)
  {
    ROS_ERROR("[Localization:] Error deleting the blocking file.");
  }

  return output;

}

