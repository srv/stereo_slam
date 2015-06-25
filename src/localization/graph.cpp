#include "localization/constants.h"
#include "localization/graph.h"
#include "common/tools.h"

using namespace tools;

namespace slam
{

  Graph::Graph(LoopClosing* loop_closing) : loop_closing_(loop_closing)
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

  void Graph::run()
  {
    ros::Rate r(500);
    while(ros::ok())
    {
      if(checkNewFrameInQueue())
      {
        processNewFrame();
      }
      r.sleep();
    }
  }

  void Graph::addFrameToQueue(Frame frame)
  {
    mutex::scoped_lock lock(mutex_frame_queue_);
    frame_queue_.push_back(frame);
  }

  bool Graph::checkNewFrameInQueue()
  {
    mutex::scoped_lock lock(mutex_frame_queue_);
    return(!frame_queue_.empty());
  }

  void Graph::processNewFrame()
  {
    // Get the frame
    Frame frame;
    {
      mutex::scoped_lock lock(mutex_frame_queue_);
      frame = frame_queue_.front();
      frame_queue_.pop_front();
    }

    // Add the vertex to the graph
    int id = addVertex(frame.getEstimatedPose(), frame.getInliers());
    frame.setId(id);

    // Get its N closest neighbors (by distance)
    frame.setGraphNeighbors( findClosestNeighbors(id) );

    // Send the frame to loop closing thread
    loop_closing_->addFrameToQueue(frame);

    // Save graph to file
    saveToFile();
  }

  int Graph::addVertex(tf::Transform pose, int inliers)
  {
    mutex::scoped_lock lock(mutex_graph_);

    // Convert pose for graph
    Eigen::Isometry3d vertex_pose = Tools::tfToIsometry(pose);

    // Set node id equal to graph size
    int id = graph_optimizer_.vertices().size();

    // Build the vertex
    g2o::VertexSE3* cur_vertex = new g2o::VertexSE3();
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
      // Get last vertex
      g2o::VertexSE3* prev_vertex = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[id - 1]);

      // Add the vertex
      graph_optimizer_.addVertex(cur_vertex);

      // When graph has been initialized get the transform between current and previous vertices
      // and save it as an edge.
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
    mutex::scoped_lock lock(mutex_graph_);

    // Get the vertices
    g2o::VertexSE3* v_i = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[i]);
    g2o::VertexSE3* v_j = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[j]);

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
    mutex::scoped_lock lock(mutex_graph_);
    graph_optimizer_.initializeOptimization();
    graph_optimizer_.optimize(20);
    ROS_INFO_STREAM("[Localization:] Optimization done in graph with " << graph_optimizer_.vertices().size() << " vertices.");
  }

  vector<int> Graph::findClosestNeighbors(int vertex_id)
  {
    mutex::scoped_lock lock(mutex_graph_);

    // Init
    vector<int> neighbors;
    const int discart_first_n = 10;
    int best_n = 3;

    // Get the vertex pose
    if (vertex_id < 0) return neighbors;
    g2o::VertexSE3* last_vertex =  dynamic_cast<g2o::VertexSE3*>
          (graph_optimizer_.vertices()[vertex_id]);
    tf::Transform last_graph_pose = Tools::getVertexPose(last_vertex);

    // Loop thought all the other nodes
    vector< pair< int,double > > neighbor_distances;
    for (int i=vertex_id-discart_first_n-1; i>=0; i--)
    {
      // Get the node pose
      g2o::VertexSE3* cur_vertex =  dynamic_cast<g2o::VertexSE3*>
              (graph_optimizer_.vertices()[i]);
      tf::Transform cur_pose = Tools::getVertexPose(cur_vertex);
      double dist = Tools::poseDiff(cur_pose, last_graph_pose);
      neighbor_distances.push_back(make_pair(i, dist));
    }

    // Exit if no neighbors
    if (neighbor_distances.size() == 0) return neighbors;

    // Sort the neighbors
    sort(neighbor_distances.begin(), neighbor_distances.end(), Tools::sortByDistance);

    // First neighbor
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

  void Graph::saveToFile()
  {
    string lock_file, vertices_file, edges_file;
    vertices_file = WORKING_DIRECTORY + "graph_vertices.txt";
    edges_file = WORKING_DIRECTORY + "graph_edges.txt";
    lock_file = WORKING_DIRECTORY + ".graph.lock";

    // Wait until lock file has been released
    while(fs::exists(lock_file));

    // Create a locking element
    fstream f_lock(lock_file.c_str(), ios::out | ios::trunc);

    // Open to append
    fstream f_vertices(vertices_file.c_str(), ios::out | ios::trunc);
    fstream f_edges(edges_file.c_str(), ios::out | ios::trunc);

    mutex::scoped_lock lock(mutex_graph_);

    // Output the vertices file
    for (uint i=0; i<graph_optimizer_.vertices().size(); i++)
    {
      g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[i]);
      tf::Transform pose = Tools::getVertexPose(v)*camera2odom_;
      f_vertices << fixed <<
            setprecision(6) <<
            i << "," <<
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
    for ( g2o::OptimizableGraph::EdgeSet::iterator it=graph_optimizer_.edges().begin();
          it!=graph_optimizer_.edges().end(); it++)
    {
      g2o::EdgeSE3* e = dynamic_cast<g2o::EdgeSE3*> (*it);
      if (e)
      {
        g2o::VertexSE3* v_0 = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[e->vertices()[0]->id()]);
        g2o::VertexSE3* v_1 = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[e->vertices()[1]->id()]);
        tf::Transform pose_0 = Tools::getVertexPose(v_0)*camera2odom_;
        tf::Transform pose_1 = Tools::getVertexPose(v_1)*camera2odom_;

        // Extract the inliers
        Eigen::Matrix<double, 6, 6> information = e->information();
        int inliers = 0;
        if (information(0,0) > 0.0001)
          inliers = (int)(1/information(0,0));

        // Write
        f_edges <<
              e->vertices()[0]->id() << "," <<
              e->vertices()[1]->id() << "," <<
              inliers << "," <<
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
      }
    }
    f_edges.close();

    // Un-lock
    f_lock.close();
    int ret_code = remove(lock_file.c_str());
    if (ret_code != 0)
      ROS_ERROR("[Localization:] Error deleting the locking file.");
  }

} //namespace slam