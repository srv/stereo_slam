#include "constants.h"
#include "graph.h"
#include "cluster.h"
#include "tools.h"

using namespace tools;

namespace slam
{

  Graph::Graph(LoopClosing* loop_closing) : frame_id_(-1), loop_closing_(loop_closing)
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

    // Remove locking file if exists
    string lock_file = WORKING_DIRECTORY + ".graph.lock";
    if (fs::exists(lock_file))
      remove(lock_file.c_str());

    // Advertise topics
    ros::NodeHandle nhp("~");
    pose_pub_ = nhp.advertise<nav_msgs::Odometry>("graph_camera_odometry", 1);
    graph_pub_ = nhp.advertise<stereo_slam::GraphPoses>("graph_poses", 2);
  }

  void Graph::run()
  {
    ros::Rate r(50);
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

    // The clusters of this frame
    vector< vector<int> > clusters = frame.getClusters();

    // Frame id
    frame_id_ = frame.getId();

    // Save the frame
    saveFrame(frame);

    // Save the frame timestamp
    frame_stamps_.push_back(frame.getTimestamp());

    // Extract sift
    cv::Mat sift_desc = frame.computeSift();

    // Loop of frame clusters
    vector<int> vertex_ids;
    vector<Cluster> clusters_to_close_loop;
    vector<Eigen::Vector4f> cluster_centroids = frame.getClusterCentroids();
    vector<cv::Point3f> points = frame.getCameraPoints();
    vector<cv::KeyPoint> kp_l = frame.getLeftKp();
    vector<cv::KeyPoint> kp_r = frame.getRightKp();
    tf::Transform camera_pose = frame.getCameraPose();
    cv::Mat orb_desc = frame.getLeftDesc();
    for (uint i=0; i<clusters.size(); i++)
    {
      // Correct cluster pose with the last graph update
      tf::Transform cluster_pose = Tools::transformVector4f(cluster_centroids[i], camera_pose);
      initial_cluster_pose_history_.push_back(cluster_pose);

      // Add cluster to the graph
      int id = addVertex(cluster_pose);

      // Store information
      cluster_frame_relation_.push_back( make_pair(id, frame_id_) );
      local_cluster_poses_.push_back( Tools::vector4fToTransform(cluster_centroids[i]) );
      vertex_ids.push_back(id);

      // Build cluster
      cv::Mat c_desc_orb, c_desc_sift;
      vector<cv::KeyPoint> c_kp_l, c_kp_r;
      vector<cv::Point3f> c_points;
      for (uint j=0; j<clusters[i].size(); j++)
      {
        int idx = clusters[i][j];
        c_kp_l.push_back(kp_l[idx]);
        c_kp_r.push_back(kp_r[idx]);
        c_points.push_back(points[idx]);
        c_desc_orb.push_back(orb_desc.row(idx));
        c_desc_sift.push_back(sift_desc.row(idx));
      }
      Cluster cluster(id, frame_id_, camera_pose, c_kp_l, c_kp_r, c_desc_orb, c_desc_sift, c_points);
      clusters_to_close_loop.push_back(cluster);
    }

    // Send the new clusters to the loop closing thread
    for (uint i=0; i<clusters_to_close_loop.size(); i++)
      loop_closing_->addClusterToQueue(clusters_to_close_loop[i]);

    // Add edges between clusters of the same frame
    if (clusters.size() > 1)
    {
      // Retrieve all possible combinations
      vector< vector<int> > combinations = createComb(vertex_ids);

      for (uint i=0; i<combinations.size(); i++)
      {
        int id_a = vertex_ids[combinations[i][0]];
        int id_b = vertex_ids[combinations[i][1]];

        tf::Transform pose_a = getVertexPose(id_a);
        tf::Transform pose_b = getVertexPose(id_b);

        tf::Transform edge = pose_a.inverse() * pose_b;
        addEdge(id_a, id_b, edge, LC_MAX_INLIERS);
      }
    }

    // Connect this frame with the previous
    if (frame_id_ > 0)
    {
      vector<int> prev_frame_vertices;
      getFrameVertices(frame_id_ - 1, prev_frame_vertices);

      // Connect only the closest vertices between the two frames
      double min_dist = DBL_MAX;
      vector<int> closest_vertices;
      vector<tf::Transform> closest_poses;
      for (uint i=0; i<vertex_ids.size(); i++)
      {
        // The pose of this vertex
        tf::Transform cur_vertex_pose = getVertexPose(vertex_ids[i]);

        for (uint j=0; j<prev_frame_vertices.size(); j++)
        {
          tf::Transform prev_vertex_pose = getVertexPose(prev_frame_vertices[j]);

          double dist = Tools::poseDiff3D(cur_vertex_pose, prev_vertex_pose);
          if (dist < min_dist)
          {
            closest_vertices.clear();
            closest_vertices.push_back(vertex_ids[i]);
            closest_vertices.push_back(prev_frame_vertices[j]);

            closest_poses.clear();
            closest_poses.push_back(cur_vertex_pose);
            closest_poses.push_back(prev_vertex_pose);
            min_dist = dist;
          }
        }
      }

      if (closest_vertices.size() > 0)
      {
        tf::Transform edge = closest_poses[0].inverse() * closest_poses[1];
        addEdge(closest_vertices[0], closest_vertices[1], edge, frame.getInliersNumWithPreviousFrame());
      }
      else
        ROS_ERROR("[Localization:] Impossible to connect current and previous frame. Graph will have non-connected parts!");
    }

    // Save graph to file
    saveGraph();

    // Publish the graph
    publishGraph();

    // Publish camera pose
    int last_idx = -1;
    {
      mutex::scoped_lock lock(mutex_graph_);
      last_idx = graph_optimizer_.vertices().size() - 1;
    }
    tf::Transform updated_camera_pose = getVertexCameraPose(last_idx, true);
    publishCameraPose(updated_camera_pose);
  }

  tf::Transform Graph::correctClusterPose(tf::Transform initial_pose)
  {
    // Get last
    int last_idx = -1;
    {
      mutex::scoped_lock lock(mutex_graph_);
      last_idx = graph_optimizer_.vertices().size() - 1;
    }

    if (initial_cluster_pose_history_.size() > 0 && last_idx >= 0)
    {
      tf::Transform last_graph_pose = getVertexPose(last_idx);
      tf::Transform last_graph_initial = initial_cluster_pose_history_.at(last_idx);
      tf::Transform diff = last_graph_initial.inverse() * initial_pose;

      // Compute the corrected pose
      return last_graph_pose * diff;
    }
    else
      return initial_pose;
  }

  vector< vector<int> > Graph::createComb(vector<int> cluster_ids)
  {
    string bitmask(2, 1);
    bitmask.resize(cluster_ids.size(), 0);

    vector<int> comb;
    vector< vector<int> > combinations;
    do {
      for (uint i = 0; i < cluster_ids.size(); ++i)
      {
        if (bitmask[i]) comb.push_back(i);
      }
      combinations.push_back(comb);
      comb.clear();
    } while (prev_permutation(bitmask.begin(), bitmask.end()));

    return combinations;
  }

  int Graph::addVertex(tf::Transform pose)
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
    }
    graph_optimizer_.addVertex(cur_vertex);
    return id;
  }

  void Graph::addEdge(int i, int j, tf::Transform edge, int sigma)
  {
    mutex::scoped_lock lock(mutex_graph_);

    // Sanity check
    if (sigma > LC_MAX_INLIERS)
      sigma = LC_MAX_INLIERS;

    // Get the vertices
    g2o::VertexSE3* v_i = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[i]);
    g2o::VertexSE3* v_j = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[j]);

    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity() * (double)sigma;

    // Add the new edge to graph
    g2o::EdgeSE3* e = new g2o::EdgeSE3();
    Eigen::Isometry3d t = Tools::tfToIsometry(edge);
    e->setVertex(0, v_i);
    e->setVertex(1, v_j);
    e->setMeasurement(t);
    e->setInformation(information);
    graph_optimizer_.addEdge(e);
  }

  void Graph::update()
  {
    mutex::scoped_lock lock(mutex_graph_);

    // Optimize!
    graph_optimizer_.initializeOptimization();
    graph_optimizer_.optimize(20);

    ROS_INFO_STREAM("[Localization:] Optimization done in graph with " << graph_optimizer_.vertices().size() << " vertices.");
  }

  void Graph::findClosestVertices(int vertex_id, int window_center, int window, int best_n, vector<int> &neighbors)
  {
    // Init
    neighbors.clear();
    tf::Transform vertex_pose = getVertexPose(vertex_id);

    // Loop thought all the other nodes
    vector< pair< int,double > > neighbor_distances;
    for (uint i=0; i<graph_optimizer_.vertices().size(); i++)
    {
      if ( (int)i == vertex_id ) continue;
      if ((int)i > window_center-window && (int)i < window_center+window) continue;

      // Get the node pose
      tf::Transform cur_pose = getVertexPose(i);
      double dist = Tools::poseDiff2D(cur_pose, vertex_pose);
      neighbor_distances.push_back(make_pair(i, dist));
    }

    // Exit if no neighbors
    if (neighbor_distances.size() == 0) return;

    // Sort the neighbors
    sort(neighbor_distances.begin(), neighbor_distances.end(), Tools::sortByDistance);

    // Min number
    if ((int)neighbor_distances.size() < best_n)
      best_n = neighbor_distances.size();

    for (int i=0; i<best_n; i++)
      neighbors.push_back(neighbor_distances[i].first);
  }

  void Graph::getFrameVertices(int frame_id, vector<int> &vertices)
  {
    vertices.clear();
    for (uint i=0; i<cluster_frame_relation_.size(); i++)
    {
      if (cluster_frame_relation_[i].second == frame_id)
        vertices.push_back(cluster_frame_relation_[i].first);
    }
  }

  int Graph::getVertexFrameId(int id)
  {
    int frame_id = -1;
    for (uint i=0; i<cluster_frame_relation_.size(); i++)
    {
      if (cluster_frame_relation_[i].first == id)
      {
        frame_id = cluster_frame_relation_[i].second;
        break;
      }
    }
    return frame_id;
  }

  int Graph::getLastVertexFrameId()
  {
    // Get last
    int last_idx = -1;
    {
      mutex::scoped_lock lock(mutex_graph_);
      last_idx = graph_optimizer_.vertices().size() - 1;
    }

    return getVertexFrameId(last_idx);
  }


  tf::Transform Graph::getVertexPose(int id, bool lock)
  {
    if (lock)
    {
      mutex::scoped_lock lock(mutex_graph_);

      if( id >= 0)
      {
        g2o::VertexSE3* vertex =  dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[id]);
        return Tools::getVertexPose(vertex);
      }
      else
      {
        tf::Transform tmp;
        tmp.setIdentity();
        return tmp;
      }
    }
    else
    {
      if( id >= 0)
      {
        g2o::VertexSE3* vertex =  dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[id]);
        return Tools::getVertexPose(vertex);
      }
      else
      {
        tf::Transform tmp;
        tmp.setIdentity();
        return tmp;
      }
    }
  }

  bool Graph::getFramePose(int frame_id, tf::Transform& frame_pose)
  {
    frame_pose.setIdentity();
    vector<int> frame_vertices;
    getFrameVertices(frame_id, frame_vertices);

    if (frame_vertices.size() == 0)
    {
      return false;
    }
    else
    {
      frame_pose = getVertexCameraPose(frame_vertices[0]);
      return true;
    }
  }

  tf::Transform Graph::getVertexPoseRelativeToCamera(int id)
  {
    return local_cluster_poses_[id];
  }

  tf::Transform Graph::getVertexCameraPose(int id, bool lock)
  {
    tf::Transform vertex_pose = getVertexPose(id, lock);
    return vertex_pose * local_cluster_poses_[id].inverse();
  }

  void Graph::saveFrame(Frame frame, bool draw_clusters)
  {
    cv::Mat img;
    frame.getLeftImg().copyTo(img);
    if (img.cols == 0)
      return;

    if (draw_clusters)
    {
      // Draw the clusters
      vector< vector<int> > clusters = frame.getClusters();
      vector<cv::KeyPoint> kp = frame.getLeftKp();
      cv::RNG rng(12345);
      for (uint i=0; i<clusters.size(); i++)
      {
        cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
        for (uint j=0; j<clusters[i].size(); j++)
          cv::circle(img, kp[clusters[i][j]].pt, 5, color, -1);
      }
    }

    // Save
    string frame_id_str = Tools::convertTo5digits(frame.getId());
    string keyframe_file = WORKING_DIRECTORY + "keyframes/" + frame_id_str + ".jpg";
    cv::imwrite( keyframe_file, img );
  }

  void Graph::saveGraph()
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

    vector<int> processed_frames;

    // Output the vertices file
    for (uint i=0; i<graph_optimizer_.vertices().size(); i++)
    {
      // Is this frame processed?
      bool found = false;
      int id = getVertexFrameId(i);
      for (uint j=0; j<processed_frames.size(); j++)
      {
        if (processed_frames[j] == id)
        {
          found = true;
          break;
        }
      }
      if (found) continue;
      processed_frames.push_back(id);

      tf::Transform pose = getVertexCameraPose(i, false);//*camera2odom_;
      f_vertices << fixed <<
        setprecision(6) <<
        frame_stamps_[id] << "," <<
        id << "," <<
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
        // Get the frames corresponding to these edges
        int frame_a = Graph::getVertexFrameId(e->vertices()[0]->id());
        int frame_b = Graph::getVertexFrameId(e->vertices()[1]->id());

        if (abs(frame_a - frame_b) > 1 )
        {

          tf::Transform pose_0 = getVertexCameraPose(e->vertices()[0]->id(), false);//*camera2odom_;
          tf::Transform pose_1 = getVertexCameraPose(e->vertices()[1]->id(), false);//*camera2odom_;

          // Extract the inliers
          Eigen::Matrix<double, 6, 6> information = e->information();
          int inliers = 0;
          if (information(0,0) > 0.0001)
            inliers = (int)information(0,0);

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
    }
    f_edges.close();

    // Un-lock
    f_lock.close();
    int ret_code = remove(lock_file.c_str());
    if (ret_code != 0)
      ROS_ERROR("[Localization:] Error deleting the locking file.");
  }

  void Graph::publishCameraPose(tf::Transform camera_pose)
  {
    if (pose_pub_.getNumSubscribers() > 0)
    {
      nav_msgs::Odometry pose_msg;
      pose_msg.header.stamp = ros::Time::now();
      tf::poseTFToMsg(camera_pose, pose_msg.pose.pose);
      pose_pub_.publish(pose_msg);
    }
  }

  void Graph::publishGraph()
  {
    if (graph_pub_.getNumSubscribers() > 0)
    {
      mutex::scoped_lock lock(mutex_graph_);

      // Build the graph data
      vector<int> ids;
      vector<double> x, y, z, qx, qy, qz, qw;
      vector<int> processed_frames;
      for (uint i=0; i<graph_optimizer_.vertices().size(); i++)
      {
        bool found = false;
        int id = getVertexFrameId(i);
        for (uint j=0; j<processed_frames.size(); j++)
        {
          if (processed_frames[j] == id)
          {
            found = true;
            break;
          }
        }
        if (found) continue;
        processed_frames.push_back(id);

        tf::Transform pose = getVertexCameraPose(i, false);
        ids.push_back(id);
        x.push_back(pose.getOrigin().x());
        y.push_back(pose.getOrigin().y());
        z.push_back(pose.getOrigin().z());
        qx.push_back(pose.getRotation().x());
        qy.push_back(pose.getRotation().y());
        qz.push_back(pose.getRotation().z());
        qw.push_back(pose.getRotation().w());
      }

      // Publish
      stereo_slam::GraphPoses graph_msg;
      graph_msg.header.stamp = ros::Time::now();
      graph_msg.id = ids;
      graph_msg.x = x;
      graph_msg.y = y;
      graph_msg.z = z;
      graph_msg.qx = qx;
      graph_msg.qy = qy;
      graph_msg.qz = qz;
      graph_msg.qw = qw;
      graph_pub_.publish(graph_msg);
    }
  }

} //namespace slam
