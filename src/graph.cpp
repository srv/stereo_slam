#include "graph.h"
#include "cluster.h"
#include "tools.h"

namespace slam
{

  Graph::Graph(LoopClosing* loop_closing) : frame_id_(-1), loop_closing_(loop_closing)
  {
    init();
  }

  void Graph::init()
  {
    // Initialize the g2o graph optimizer
    std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linear_solver_ptr (new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>());
    std::unique_ptr<g2o::BlockSolverX> solver_ptr (new g2o::BlockSolverX(std::move(linear_solver_ptr)));
    g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    graph_optimizer_.setAlgorithm(solver);

    // Remove locking file if exists
    std::string lock_file = params_.working_directory + ".graph.lock";
    if (boost::filesystem::exists(lock_file))
      remove(lock_file.c_str());

    // Advertise topics
    ros::NodeHandle nhp("~");
    pub_graph_ = nhp.advertise<stereo_slam::GraphPoses>("graph_poses", 2);
    pub_time_graph_ = nhp.advertise<stereo_slam::TimeGraph>("time_graph", 1);
    pub_robot_pose_ = nhp.advertise<nav_msgs::Odometry>("graph_robot_odometry", 1);
    pub_camera_pose_ = nhp.advertise<nav_msgs::Odometry>("graph_camera_odometry", 1);
    pub_num_keyframes_ = nhp.advertise<std_msgs::Int32>("keyframes", 1);
  }

  void Graph::run()
  {
    ros::Rate r(50);
    while(ros::ok())
    {
      // Process new frame
      if(checkNewFrameInQueue())
      {
        double t0 = ros::Time::now().toSec();

        processNewFrame();

        if (pub_num_keyframes_.getNumSubscribers() > 0)
        {
          std_msgs::Int32 msg;
          msg.data = boost::lexical_cast<int>(getFrameNum());
          pub_num_keyframes_.publish(msg);
        }
        if (pub_time_graph_.getNumSubscribers() > 0)
        {
          time_graph_msg_.header.stamp = ros::Time::now();
          time_graph_msg_.total = ros::Time::now().toSec() - t0;
          pub_time_graph_.publish(time_graph_msg_);
        }
      }
      r.sleep();
    }
  }

  void Graph::addFrameToQueue(Frame frame)
  {
    boost::mutex::scoped_lock lock(mutex_frame_queue_);
    frame_queue_.push_back(frame);
  }

  bool Graph::checkNewFrameInQueue()
  {
    boost::mutex::scoped_lock lock(mutex_frame_queue_);
    return(!frame_queue_.empty());
  }

  void Graph::processNewFrame()
  {
    // Get the frame
    Frame frame;
    {
      boost::mutex::scoped_lock lock(mutex_frame_queue_);
      frame = frame_queue_.front();
      frame_queue_.pop_front();
    }

    // The clusters of this frame
    std::vector< std::vector<int> > clusters = frame.getClusters();

    // Frame id
    frame_id_ = frame.getId();

    // Save the frame
    saveFrame(frame);

    // Save the frame timestamp
    frame_stamps_.push_back(frame.getTimestamp());

    // Extract sift
    double t1 = ros::Time::now().toSec();
    cv::Mat sift_desc = frame.computeSift();
    time_graph_msg_.compute_sift_desc = ros::Time::now().toSec() - t1;

    // Loop of frame clusters
    double t2 = ros::Time::now().toSec();
    std::vector<int> vertex_ids;
    std::vector<Cluster> clusters_to_close_loop;
    std::vector<Eigen::Vector4f> cluster_centroids = frame.getClusterCentroids();
    std::vector<cv::Point3f> points = frame.getCameraPoints();
    std::vector<cv::KeyPoint> kp_l = frame.getLeftKp();
    std::vector<cv::KeyPoint> kp_r = frame.getRightKp();
    tf::Transform camera_pose = frame.getCameraPose();
    cv::Mat orb_desc = frame.getLeftDesc();
    for (uint i=0; i<clusters.size(); i++)
    {
      // Correct cluster pose with the last graph update
      tf::Transform cluster_pose = tools::Tools::transformVector4f(cluster_centroids[i], camera_pose);
      initial_cluster_pose_history_.push_back(cluster_pose);

      // Add cluster to the graph
      int id = addVertex(cluster_pose);

      // Store information
      cluster_frame_relation_.push_back(std::make_pair(id, frame_id_) );
      local_cluster_poses_.push_back(tools::Tools::vector4fToTransform(cluster_centroids[i]) );
      vertex_ids.push_back(id);

      // Build cluster
      cv::Mat c_desc_orb, c_desc_sift;
      std::vector<cv::KeyPoint> c_kp_l, c_kp_r;
      std::vector<cv::Point3f> c_points;
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

    time_graph_msg_.add_clusters = ros::Time::now().toSec() - t2;

    // Add edges between clusters of the same frame
    double t3 = ros::Time::now().toSec();
    if (clusters.size() > 1)
    {
      // Retrieve all possible combinations
      std::vector< std::vector<int> > combinations = createComb(vertex_ids);

      for (uint i=0; i<combinations.size(); i++)
      {
        int id_a = vertex_ids[combinations[i][0]];
        int id_b = vertex_ids[combinations[i][1]];

        tf::Transform pose_a = getVertexPose(id_a);
        tf::Transform pose_b = getVertexPose(id_b);

        tf::Transform edge = pose_a.inverse() * pose_b;
        cv::Mat eye = cv::Mat::eye(6, 6, CV_64F);
        addEdge(id_a, id_b, edge, eye, 0);
      }
    }

    // Connect this frame with the previous
    if (frame_id_ > 0)
    {
      std::vector<int> prev_frame_vertices;
      getFrameVertices(frame_id_ - 1, prev_frame_vertices);

      // Connect only the closest vertices between the two frames
      double min_dist = DBL_MAX;
      std::vector<int> closest_vertices;
      std::vector<tf::Transform> closest_poses;
      for (uint i=0; i<vertex_ids.size(); i++)
      {
        // The pose of this vertex
        tf::Transform cur_vertex_pose = getVertexPose(vertex_ids[i]);

        for (uint j=0; j<prev_frame_vertices.size(); j++)
        {
          tf::Transform prev_vertex_pose = getVertexPose(prev_frame_vertices[j]);

          double dist = tools::Tools::poseDiff3D(cur_vertex_pose, prev_vertex_pose);
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
        cv::Mat eye = cv::Mat::eye(6, 6, CV_64F);
        tf::Transform edge = closest_poses[0].inverse() * closest_poses[1];
        addEdge(closest_vertices[0], closest_vertices[1], edge, frame.getSigmaWithPreviousFrame(), frame.getInliersNumWithPreviousFrame());

        int frame_i = Graph::getVertexFrameId(closest_vertices[0]);
        int frame_j = Graph::getVertexFrameId(closest_vertices[1]);
        Edge ee(frame_i, frame_j, frame.getInliersNumWithPreviousFrame());
        edges_information_.push_back(ee);
      }
      else
        ROS_ERROR("[Localization:] Impossible to connect current and previous frame. Graph will have non-connected parts!");
    }
    time_graph_msg_.add_edges = ros::Time::now().toSec() - t3;

    double t4 = ros::Time::now().toSec();

    // Save graph to file
    saveGraph();

    // Publish the graph
    publishGraph();

    time_graph_msg_.save_and_publish_graph = ros::Time::now().toSec() - t4;

    // Publish camera pose
    int last_idx = -1;
    {
      boost::mutex::scoped_lock lock(mutex_graph_);
      last_idx = graph_optimizer_.vertices().size() - 1;
    }
    tf::Transform updated_camera_pose = getVertexCameraPose(last_idx, true);
    publishUpdatedPose(updated_camera_pose);
  }

  tf::Transform Graph::correctClusterPose(tf::Transform initial_pose)
  {
    // Get last
    int last_idx = -1;
    {
      boost::mutex::scoped_lock lock(mutex_graph_);
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

  std::vector< std::vector<int> > Graph::createComb(std::vector<int> cluster_ids)
  {
    std::string bitmask(2, 1);
    bitmask.resize(cluster_ids.size(), 0);

    std::vector<int> comb;
    std::vector< std::vector<int> > combinations;
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
    boost::mutex::scoped_lock lock(mutex_graph_);

    // Convert pose for graph
    Eigen::Isometry3d vertex_pose = tools::Tools::tfToIsometry(pose);

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

  void Graph::addEdge(int i, int j, tf::Transform edge, cv::Mat sigma, int inliers)
  {
    boost::mutex::scoped_lock lock(mutex_graph_);

    // Store edge information
    int frame_i = Graph::getVertexFrameId(i);
    int frame_j = Graph::getVertexFrameId(j);
    bool lc_found = false;
    for (uint i=0; i<edges_information_.size(); i++)
    {
      Edge ee = edges_information_[i];
      if ( (ee.vertice_a == frame_i && ee.vertice_b == frame_j) ||
           (ee.vertice_a == frame_j && ee.vertice_b == frame_i) )
      {
        lc_found = true;
        edges_information_[i].inliers = ee.inliers + inliers;
        break;
      }
    }

    if (!lc_found)
    {
      Edge e_info(frame_i, frame_j, inliers);
      edges_information_.push_back(e_info);
    }

    // Eigen::Matrix<double, 6, 6> information;
    // information(0,0) = sigma.at<double>(0,0);
    // information(0,1) = sigma.at<double>(0,1);
    // information(0,2) = sigma.at<double>(0,2);
    // information(0,3) = sigma.at<double>(0,3);
    // information(0,4) = sigma.at<double>(0,4);
    // information(0,5) = sigma.at<double>(0,5);
    // information(1,0) = sigma.at<double>(1,0);
    // information(1,1) = sigma.at<double>(1,1);
    // information(1,2) = sigma.at<double>(1,2);
    // information(1,3) = sigma.at<double>(1,3);
    // information(1,4) = sigma.at<double>(1,4);
    // information(1,5) = sigma.at<double>(1,5);
    // information(2,0) = sigma.at<double>(2,0);
    // information(2,1) = sigma.at<double>(2,1);
    // information(2,2) = sigma.at<double>(2,2);
    // information(2,3) = sigma.at<double>(2,3);
    // information(2,4) = sigma.at<double>(2,4);
    // information(2,5) = sigma.at<double>(2,5);
    // information(3,0) = sigma.at<double>(3,0);
    // information(3,1) = sigma.at<double>(3,1);
    // information(3,2) = sigma.at<double>(3,2);
    // information(3,3) = sigma.at<double>(3,3);
    // information(3,4) = sigma.at<double>(3,4);
    // information(3,5) = sigma.at<double>(3,5);
    // information(4,0) = sigma.at<double>(4,0);
    // information(4,1) = sigma.at<double>(4,1);
    // information(4,2) = sigma.at<double>(4,2);
    // information(4,3) = sigma.at<double>(4,3);
    // information(4,4) = sigma.at<double>(4,4);
    // information(4,5) = sigma.at<double>(4,5);
    // information(5,0) = sigma.at<double>(5,0);
    // information(5,1) = sigma.at<double>(5,1);
    // information(5,2) = sigma.at<double>(5,2);
    // information(5,3) = sigma.at<double>(5,3);
    // information(5,4) = sigma.at<double>(5,4);
    // information(5,5) = sigma.at<double>(5,5);

    // Get the vertices
    g2o::VertexSE3* v_i = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[i]);
    g2o::VertexSE3* v_j = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[j]);

    // Add the new edge to graph
    g2o::EdgeSE3* e = new g2o::EdgeSE3();
    Eigen::Isometry3d t = tools::Tools::tfToIsometry(edge);
    e->setVertex(0, v_i);
    e->setVertex(1, v_j);
    e->setMeasurement(t);
    // e->setInformation(information);

    graph_optimizer_.addEdge(e);
  }

  void Graph::update()
  {
    boost::mutex::scoped_lock lock(mutex_graph_);

    // Optimize!
    graph_optimizer_.initializeOptimization();
    graph_optimizer_.optimize(20);

    ROS_INFO_STREAM("[Localization:] Optimization done in graph with " << graph_optimizer_.vertices().size() << " vertices.");
  }

  void Graph::findClosestVertices(int vertex_id, int window_center, int window, int best_n, std::vector<int> &neighbors)
  {
    // Init
    neighbors.clear();
    tf::Transform vertex_pose = getVertexPose(vertex_id);

    // Loop thought all the other nodes
    std::vector< std::pair< int,double > > neighbor_distances;
    for (uint i=0; i<graph_optimizer_.vertices().size(); i++)
    {
      if ((int)i == vertex_id ) continue;
      if ((int)i > window_center-window && (int)i < window_center+window) continue;

      // Get the node pose
      tf::Transform cur_pose = getVertexPose(i);
      double dist = tools::Tools::poseDiff2D(cur_pose, vertex_pose);
      neighbor_distances.push_back(std::make_pair(i, dist));
    }

    // Exit if no neighbors
    if (neighbor_distances.size() == 0) return;

    // Sort the neighbors
    std::sort(neighbor_distances.begin(), neighbor_distances.end(), tools::Tools::sortByDistance);

    // Min number
    if ((int)neighbor_distances.size() < best_n)
      best_n = neighbor_distances.size();

    for (int i=0; i<=best_n; i++)
      neighbors.push_back(neighbor_distances[i].first);
  }

  void Graph::getFrameVertices(int frame_id, std::vector<int> &vertices)
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
      boost::mutex::scoped_lock lock(mutex_graph_);
      last_idx = graph_optimizer_.vertices().size() - 1;
    }

    return getVertexFrameId(last_idx);
  }


  tf::Transform Graph::getVertexPose(int id, bool lock)
  {
    if (lock)
    {
      boost::mutex::scoped_lock lock(mutex_graph_);

      if( id >= 0)
      {
        g2o::VertexSE3* vertex =  dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[id]);
        return tools::Tools::getVertexPose(vertex);
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
        return tools::Tools::getVertexPose(vertex);
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
    std::vector<int> frame_vertices;
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

  void Graph::saveFrame(Frame frame)
  {
    cv::Mat l_img, r_img, c_img;
    frame.getLeftImg().copyTo(l_img);
    frame.getRightImg().copyTo(r_img);
    frame.getLeftImg().copyTo(c_img);
    if (l_img.cols == 0 || r_img.cols == 0)
      return;

    std::string frame_id_str = tools::Tools::convertTo5digits(frame.getId());

    // Save keyframe
    std::string l_kf = params_.working_directory + "keyframes/" + frame_id_str + "_left.jpg";
    std::string r_kf = params_.working_directory + "keyframes/" + frame_id_str + "_right.jpg";
    cv::imwrite(l_kf, l_img);
    cv::imwrite(r_kf, r_img);

    // Save keyframe with clusters
    std::vector< std::vector<int> > clusters = frame.getClusters();
    std::vector<cv::KeyPoint> kp = frame.getLeftKp();
    cv::RNG rng(12345);
    for (uint i=0; i<clusters.size(); i++)
    {
      cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
      for (uint j=0; j<clusters[i].size(); j++)
        cv::circle(c_img, kp[clusters[i][j]].pt, 5, color, -1);
    }
    std::string clusters_file = params_.working_directory + "clusters/" + frame_id_str + ".jpg";
    cv::imwrite(clusters_file, c_img);
  }

  void Graph::saveGraph()
  {
    std::string lock_file, vertices_file, edges_file;
    vertices_file = params_.working_directory + "graph_vertices.txt";
    edges_file = params_.working_directory + "graph_edges.txt";
    lock_file = params_.working_directory + ".graph.lock";

    // Wait until lock file has been released
    while(boost::filesystem::exists(lock_file));

    // Create a locking element
    std::fstream f_lock(lock_file.c_str(), std::ios::out | std::ios::trunc);

    // Open to append
    std::fstream f_vertices(vertices_file.c_str(), std::ios::out | std::ios::trunc);
    std::fstream f_edges(edges_file.c_str(), std::ios::out | std::ios::trunc);

    boost::mutex::scoped_lock lock(mutex_graph_);

    // First line
    f_vertices << "% timestamp,frame id,x,y,z,qx,qy,qz,qw" << std::endl;

    std::vector<int> processed_frames;

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

      tf::Transform pose = getVertexCameraPose(i, false) * camera2robot_;
      f_vertices << std::fixed <<
        std::setprecision(9) <<
        frame_stamps_[id] << "," <<
        id << "," <<
        pose.getOrigin().x() << "," <<
        pose.getOrigin().y() << "," <<
        pose.getOrigin().z() << "," <<
        pose.getRotation().x() << "," <<
        pose.getRotation().y() << "," <<
        pose.getRotation().z() << "," <<
        pose.getRotation().w() <<  std::endl;
    }
    f_vertices.close();

    // First line
    f_edges << "% frame a,frame b,inliers,ax,ay,az,aqx,aqy,aqz,aqw,bx,by,bz,bqx,bqy,bqz,bqw" << std::endl;

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

          tf::Transform pose_0 = getVertexCameraPose(e->vertices()[0]->id(), false) * camera2robot_;
          tf::Transform pose_1 = getVertexCameraPose(e->vertices()[1]->id(), false) * camera2robot_;

          // Extract the inliers
          int inliers = 0;
          for (uint i=0; i<edges_information_.size(); i++)
          {
            Edge ee = edges_information_[i];
            if ( (ee.vertice_a == frame_a && ee.vertice_b == frame_b) ||
                 (ee.vertice_a == frame_b && ee.vertice_b == frame_a) )
            {
              inliers = edges_information_[i].inliers;
              break;
            }
          }

          // Write
          f_edges <<
            e->vertices()[0]->id() << "," <<
            e->vertices()[1]->id() << "," <<
            inliers << "," <<
            std::setprecision(9) <<
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
            pose_1.getRotation().w() << std::endl;
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

  void Graph::publishUpdatedPose(tf::Transform camera_pose)
  {
    // Transform pose from camera to robot frame
    tf::Transform robot_pose = camera_pose * camera2robot_;

    // Publish poses
    nav_msgs::Odometry pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    if (pub_camera_pose_.getNumSubscribers() > 0)
    {
      tf::poseTFToMsg(camera_pose, pose_msg.pose.pose);
      pub_camera_pose_.publish(pose_msg);
    }
    if (pub_robot_pose_.getNumSubscribers() > 0)
    {
      pose_msg.header.frame_id = params_.map_frame_id;
      pose_msg.child_frame_id = odom_frame_id_;
      tf::poseTFToMsg(robot_pose, pose_msg.pose.pose);
      pub_robot_pose_.publish(pose_msg);
    }
  }

  void Graph::publishGraph()
  {
    if (pub_graph_.getNumSubscribers() > 0)
    {
      boost::mutex::scoped_lock lock(mutex_graph_);

      // Build the graph data
      std::vector<int> ids;
      std::vector<double> x, y, z, qx, qy, qz, qw;
      std::vector<int> processed_frames;
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
      pub_graph_.publish(graph_msg);
    }
  }

} //namespace slam
