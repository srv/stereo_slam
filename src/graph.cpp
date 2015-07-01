#include "constants.h"
#include "graph.h"
#include "cluster.h"
#include "tools.h"

using namespace tools;

namespace slam
{

  Graph::Graph(LoopClosing* loop_closing) : frames_counter_(0), loop_closing_(loop_closing)
  {
    init();
  }

  void Graph::init()
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

    // Increase the counter
    frames_counter_++;

    // Extract sift
    cv::Mat sift_desc = frame.computeSift();

    // Loop of frame clusters
    vector<Eigen::Vector4f> cluster_centroids = frame.getClusterCentroids();
    vector<PointIndices> clusters = frame.getClusters();
    vector<cv::Point3f> points = frame.getCameraPoints();
    vector<cv::KeyPoint> kp = frame.getLeftKp();
    tf::Transform camera_pose = frame.getPose();
    cv::Mat orb_desc = frame.getLeftDesc();
    for (uint i=0; i<clusters.size(); i++)
    {
      // Add cluster to graph
      Eigen::Vector4f centroid = cluster_centroids[i];
      int id = addVertex(centroid);

      // Add cluster to loop_closing
      cv::Mat c_desc_orb, c_desc_sift;
      vector<cv::KeyPoint> c_kp;
      vector<cv::Point3f> c_points;
      for (uint j=0; j<clusters[i].indices.size(); j++)
      {
        int idx = clusters[i].indices[j];
        c_kp.push_back(kp[idx]);
        c_points.push_back(points[idx]);
        c_desc_orb.push_back(orb_desc.row(idx));
        c_desc_sift.push_back(sift_desc.row(idx));
      }

      if (c_kp.size() > 10)
      {
        // Add to loop closing
        Cluster cluster(id, frames_counter_, camera_pose, c_kp, c_desc_orb, c_desc_sift, c_points);
        loop_closing_->addClusterToQueue(cluster);
      }
    }

    // // Get its N closest neighbors (by distance)
    // frame.setGraphNeighbors( findClosestNeighbors(id) );

    // // Send the frame to loop closing thread
    // loop_closing_->addFrameToQueue(frame);

    // // Save graph to file
    // saveToFile();
  }

  int Graph::addVertex(Eigen::Vector4f pose)
  {
    mutex::scoped_lock lock(mutex_graph_);

    // Convert pose for graph
    Eigen::Isometry3d vertex_pose = Tools::vector4fToIsometry(pose);

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
    }
    graph_optimizer_.addVertex(cur_vertex);
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