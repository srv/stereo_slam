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

/** \brief Advertises the graph messages
  * @return
  * \param Node handle where graph will be advertised.
  */
void slam::Graph::advertiseMsgs(ros::NodeHandle nh)
{
  // Advertise
  vertex_pub_ = nh.advertise<stereo_slam::SlamVertex>("vertex", 1);
  edge_pub_ = nh.advertise<stereo_slam::SlamEdge>("edge", 1);
}


/** \brief Subscribes the graph correction messages
  * @return
  * \param Node handle where graph will be advertised.
  */
void slam::Graph::subscribeMsgs(ros::NodeHandle nh)
{
  // Subscribe to correction messages (if any)
  if (params_.correction_tp != "")
  {
    correction_sub_ = nh.subscribe<stereo_slam::Correction>(params_.correction_tp, 5, &Graph::correctionCallback, this);
  }
}


/** \brief Init the graph
  * @return
  */
bool slam::Graph::init()
{
  // Init lock
  lock_ = false;

  // Delete all the files (if any)
  string lock_file, vertices_file, edges_file;
  vertices_file = params_.save_dir + "graph_vertices.txt";
  edges_file = params_.save_dir + "graph_edges.txt";
  lock_file = params_.save_dir + ".graph.lock";
  fs::remove(vertices_file);
  fs::remove(edges_file);
  fs::remove(lock_file);

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


/** \brief Correction callback
  * @return
  * \param correction_msg message of type stereo_slam::Correction
  */
void slam::Graph::correctionCallback(const stereo_slam::Correction::ConstPtr& correction_msg)
{
  // Edge between nodes
  int node_0 = correction_msg->node_0;
  int node_1 = correction_msg->node_1;

  // Extract the correction
  tf::Vector3 t(correction_msg->x, correction_msg->y, correction_msg->z);
  tf::Quaternion q(correction_msg->qx, correction_msg->qy, correction_msg->qz, correction_msg->qw);
  tf::Transform pose(q, t);

  // Search this edge and delete it
  g2o::EdgeSE3* e;
  slam::Vertex* v0;
  slam::Vertex* v1;
  bool found = false;
  for ( g2o::OptimizableGraph::EdgeSet::iterator it=graph_optimizer_.edges().begin();
        it!=graph_optimizer_.edges().end(); it++)
  {
    e = dynamic_cast<g2o::EdgeSE3*> (*it);
    if (e)
    {
      v0 = dynamic_cast<slam::Vertex*>(e->vertex(0));
      v1 = dynamic_cast<slam::Vertex*>(e->vertex(1));
      if (v0->id() == node_0 && v1->id() == node_1)
      {
        found = true;
        break;
      }
    }
  }

  if (!found) return;

  // Lock the graph
  while(lock_) {}
    lock_ = true;

  // Delete this edge
  if (!graph_optimizer_.removeEdge(e)) return;

  // Replace the edge
  g2o::EdgeSE3* e_new = new g2o::EdgeSE3();
  e_new->setVertex(0, v0);
  e_new->setVertex(1, v1);
  e_new->setMeasurement(Tools::tfToIsometry(pose));
  graph_optimizer_.addEdge(e_new);

  // Unlock the graph
  lock_ = false;

  // Log
  ROS_INFO_STREAM("[Localization:] Correction applied between vertices " << node_0 << " and " << node_1 << ".");

  // update
  update();
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
  * @return the vertex id
  * \param Last estimated pose.
  * \param Last corrected pose.
  * \param Timestamp for the current odometry.
  */
int slam::Graph::addVertex(tf::Transform pose,
                           tf::Transform pose_corrected,
                           ros::Time timestamp)
{
  // Save the original odometry for this new node
  odom_history_.push_back(make_pair(pose, timestamp.toSec()));

  // Add the node
  return addVertex(pose_corrected, timestamp);
}


/** \brief Add new vertex into the graph
  * @return the vertex id
  * \param Last corrected odometry pose.
  * \param Current read odometry.
  * \param Timestamp for the current odometry.
  */
int slam::Graph::addVertex(tf::Transform pose, ros::Time timestamp)
{
  // Lock the graph
  while(lock_) {}
    lock_ = true;

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

  // Unlock the graph
  lock_ = false;

  // Publish
  if (vertex_pub_.getNumSubscribers() > 0)
  {
    stereo_slam::SlamVertex vertex_msg;
    vertex_msg.header.stamp = timestamp;
    vertex_msg.node_id = id;
    vertex_msg.x = pose.getOrigin().x();
    vertex_msg.y = pose.getOrigin().y();
    vertex_msg.z = pose.getOrigin().z();
    vertex_msg.qx = pose.getRotation().x();
    vertex_msg.qy = pose.getRotation().y();
    vertex_msg.qz = pose.getRotation().z();
    vertex_msg.qw = pose.getRotation().w();
    vertex_pub_.publish(vertex_msg);
  }

  return id;
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

  // Lock the graph
  while(lock_) {}
    lock_ = true;

  // Get the vertices
  slam::Vertex* v_i = dynamic_cast<slam::Vertex*>(graph_optimizer_.vertices()[i]);
  slam::Vertex* v_j = dynamic_cast<slam::Vertex*>(graph_optimizer_.vertices()[j]);

  // Add the new edge to graph
  g2o::EdgeSE3* e = new g2o::EdgeSE3();
  Eigen::Isometry3d t = Tools::tfToIsometry(edge.inverse());
  e->setVertex(0, v_i);
  e->setVertex(1, v_j);
  e->setMeasurement(t);
  graph_optimizer_.addEdge(e);

  // Unlock the graph
  lock_ = false;

  // Publish
  if (edge_pub_.getNumSubscribers() > 0)
  {
    tf::Transform edge_inv = edge.inverse();
    stereo_slam::SlamEdge edge_msg;
    edge_msg.header.stamp = ros::Time::now();
    edge_msg.node_0 = i;
    edge_msg.node_1 = j;
    edge_msg.x = edge_inv.getOrigin().x();
    edge_msg.y = edge_inv.getOrigin().y();
    edge_msg.z = edge_inv.getOrigin().z();
    edge_msg.qx = edge_inv.getRotation().x();
    edge_msg.qy = edge_inv.getRotation().y();
    edge_msg.qz = edge_inv.getRotation().z();
    edge_msg.qw = edge_inv.getRotation().w();
    edge_pub_.publish(edge_msg);
  }
}


/** \brief Updates vertex estimate
  * @return
  * \param Id vertex.
  * \param New estimate.
  */
void slam::Graph::setVertexEstimate(int vertex_id, tf::Transform pose)
{
  // Lock the graph
  while(lock_) {}
    lock_ = true;

  dynamic_cast<slam::Vertex*>(graph_optimizer_.vertices()[vertex_id])->setEstimate(Tools::tfToIsometry(pose));

  // Unlock the graph
  lock_ = false;
}


/** \brief Update the graph
  */
void slam::Graph::update()
{
  // Lock the graph
  while(lock_) {}
    lock_ = true;

  graph_optimizer_.initializeOptimization();
  graph_optimizer_.optimize(params_.go2_opt_max_iter);
  ROS_INFO_STREAM("[Localization:] Optimization done in graph with " << graph_optimizer_.vertices().size() << " vertices.");

  // Unlock the graph
  lock_ = false;
}


/** \brief Save the optimized graph into a file with the same format than odometry_msgs.
  * @return
  */
bool slam::Graph::saveToFile(tf::Transform camera2odom)
{
  string lock_file, vertices_file, edges_file;
  vertices_file = params_.save_dir + "graph_vertices.txt";
  edges_file = params_.save_dir + "graph_edges.txt";
  lock_file = params_.save_dir + ".graph.lock";

  // Wait until lock file has been released
  while(fs::exists(lock_file));

  // Create a locking element
  fstream f_lock(lock_file.c_str(), ios::out | ios::trunc);

  // Open to append
  fstream f_vertices(vertices_file.c_str(), ios::out | ios::trunc);
  fstream f_edges(edges_file.c_str(), ios::out | ios::trunc);

  // Output the vertices file
  for (unsigned int i=0; i<graph_optimizer_.vertices().size(); i++)
  {
    // Get timestamp
    double timestamp = odom_history_.at(i).second;

    slam::Vertex* v = dynamic_cast<slam::Vertex*>(graph_optimizer_.vertices()[i]);
    tf::Transform pose = Tools::getVertexPose(v)*camera2odom;
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
        tf::Transform pose_0 = Tools::getVertexPose(v_0)*camera2odom;
        tf::Transform pose_1 = Tools::getVertexPose(v_1)*camera2odom;

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

  // Un-lock
  f_lock.close();
  int ret_code = remove(lock_file.c_str());
  if (ret_code != 0)
  {
    ROS_ERROR("[Localization:] Error deleting the locking file.");
    return false;
  }

  return true;
}


/** \brief Get the number of vertices of the graph
  * @return
  */
int slam::Graph::numNodes()
{
  return (int)graph_optimizer_.vertices().size();
}


/** \brief Get the number of edges of the graph
  * @return
  */
int slam::Graph::numLoopClosures()
{
  int num_vertices = (int)graph_optimizer_.vertices().size();
  int num_edges = (int)graph_optimizer_.edges().size();
  if (num_edges > 0)
  {
    return num_edges - (num_vertices-1);
  }
  else
  {
    return 0;
  }
}