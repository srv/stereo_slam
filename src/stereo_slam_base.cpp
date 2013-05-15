#include "stereo_slam_base.h"
#include <boost/shared_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <libpq-fe.h>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include "postgresql_interface.h"
#include "utils.h"

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
namespace enc = sensor_msgs::image_encodings;

/** \brief Class constructor. Reads node parameters and initialize some properties.
  * @return 
  * \param nh public node handler
  * \param nhp private node handler
  */
stereo_slam::StereoSlamBase::StereoSlamBase(
  ros::NodeHandle nh, ros::NodeHandle nhp) : nh_(nh), nh_private_(nhp)
{
  // Read the node parameters
  readParameters();

  // Initialize the stereo slam
  initializeStereoSlam();
}

/** \brief Messages callback. This function is called when syncronized odometry and image
  * message are received.
  * @return 
  * \param odom_msg ros odometry message of type nav_msgs::Odometry
  * \param left_msg ros image message of type sensor_msgs::Image
  */
void stereo_slam::StereoSlamBase::msgsCallback(
                                  const nav_msgs::Odometry::ConstPtr& odom_msg,
                                  const sensor_msgs::ImageConstPtr& l_img,
                                  const sensor_msgs::ImageConstPtr& r_img,
                                  const sensor_msgs::CameraInfoConstPtr& l_info,
                                  const sensor_msgs::CameraInfoConstPtr& r_info)
{
  // Set camera info
  stereo_camera_model_.fromCameraInfo(l_info, r_info);
  const cv::Mat P(3,4, CV_64FC1, const_cast<double*>(l_info->P.data()));
  camera_matrix_ = P.colRange(cv::Range(0,3)).clone();

  // Get the current odometry for these images
  tf::Vector3 tf_trans( odom_msg->pose.pose.position.x,
                        odom_msg->pose.pose.position.y,
                        odom_msg->pose.pose.position.z);
  tf::Quaternion tf_q ( odom_msg->pose.pose.orientation.x,
                        odom_msg->pose.pose.orientation.y,
                        odom_msg->pose.pose.orientation.z,
                        odom_msg->pose.pose.orientation.w);
  tf::Transform current_pose(tf_q, tf_trans);
  tf::Transform corrected_pose = current_pose;

  // Compute the corrected pose with the optimized graph
  if (pose_history_.size() > 0 && last_node_optimized_ >= 0)
  {
    // Compute the tf between last original pose before optimization and current.
    tf::Transform last_original_pose = pose_history_.at(last_node_optimized_);
    tf::Transform diff = last_original_pose.inverse() * current_pose;

    // Get the last optimized pose
    g2o::VertexSE3* last_node =  dynamic_cast<g2o::VertexSE3*>
          (graph_optimizer_.vertices()[last_node_optimized_]);
    tf::Transform last_optimized_pose = stereo_slam::Utils::getNodePose(last_node);

    // Compute the corrected pose
    corrected_pose = last_optimized_pose * diff;
  }

  // Check if difference between images is larger than maximum displacement
  if (stereo_slam::Utils::poseDiff(corrected_pose, previous_pose_) > min_displacement_
      || first_message_)
  {
    // The image properties and pose must be saved into the database
    
    // Convert message to cv::Mat
    cv_bridge::CvImagePtr l_ptr, r_ptr;
    try
    {
      l_ptr = cv_bridge::toCvCopy(l_img, enc::BGR8);
      r_ptr = cv_bridge::toCvCopy(r_img, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("[StereoSlam:] cv_bridge exception: %s", e.what());
      return;
    }

    // Extract keypoints and descriptors of images
    std::vector<cv::KeyPoint> l_kp, r_kp;
    cv::Mat l_desc = cv::Mat_<std::vector<float> >();
    cv::Mat r_desc = cv::Mat_<std::vector<float> >();
    stereo_slam::Utils::keypointDetector(l_ptr->image, l_kp, descriptor_type_);
    stereo_slam::Utils::keypointDetector(r_ptr->image, r_kp, descriptor_type_);
    stereo_slam::Utils::descriptorExtraction(l_ptr->image, l_kp, l_desc, descriptor_type_);
    stereo_slam::Utils::descriptorExtraction(r_ptr->image, r_kp, r_desc, descriptor_type_);

    // Find matching between stereo images
    std::vector<cv::DMatch> matches;
    stereo_slam::Utils::thresholdMatching(l_desc, r_desc, matches, descriptor_threshold_);

    // Compute 3D points
    std::vector<cv::Point2f> matched_keypoints;
    std::vector<cv::Point3f> matched_3d_points;
    cv::Mat matched_descriptors;
    for (size_t i = 0; i < matches.size(); ++i)
    {
      int index_left = matches[i].queryIdx;
      int index_right = matches[i].trainIdx;
      cv::Point3d world_point;
      stereo_slam::Utils::calculate3DPoint( stereo_camera_model_,
                                                    l_kp[index_left].pt,
                                                    r_kp[index_right].pt,
                                                    world_point);
      matched_3d_points.push_back(world_point);
      matched_keypoints.push_back(l_kp[index_left].pt);
      matched_descriptors.push_back(l_desc.row(index_left));
    }

    // Transform data to std::vector for database
    std::vector< std::vector<float> > keypoints = 
    stereo_slam::Utils::cvPoint2fToStdMatrix(matched_keypoints);
    std::vector< std::vector<float> > descriptors = 
    stereo_slam::Utils::cvMatToStdMatrix(matched_descriptors);
    std::vector< std::vector<float> > points_3d = 
    stereo_slam::Utils::cvPoint3fToStdMatrix(matched_3d_points);

    // Save node data into the database
    stereo_slam::GraphNodes node_data;
    node_data.keypoints_.data() = keypoints;
    node_data.descriptors_.data() = descriptors;
    node_data.points3d_.data() = points_3d;
    if (!pg_db_ptr_thread_1_->insertIntoDatabase(&node_data))
    {
      ROS_ERROR("[StereoSlam:] Node insertion failed");
    }
    else
    {
      // Everything is ok, save the node into the graph

      // Build the pose
      Eigen::Isometry3d pose = stereo_slam::Utils::tfToEigen(corrected_pose);

      // Build the node
      g2o::VertexSE3* cur_node = new g2o::VertexSE3();
      cur_node->setId(node_data.id_.data() - 1);
      cur_node->setEstimate(pose);
      if (first_node_)
      {
        // First time, no edges.
        cur_node->setFixed(true);
        graph_optimizer_.addVertex(cur_node);
        first_node_ = false;
      }
      else
      {
        // When graph has been initialized get the transform between current and previous nodes
        // and save it as an edge

        // Get last node
        g2o::VertexSE3* prev_node = dynamic_cast<g2o::VertexSE3*>(
          graph_optimizer_.vertices()[graph_optimizer_.vertices().size() - 1]);
        graph_optimizer_.addVertex(cur_node);

        // Odometry edges
        g2o::EdgeSE3* e = new g2o::EdgeSE3();
        Eigen::Isometry3d t = prev_node->estimate().inverse() * cur_node->estimate();
        e->setVertex(0, prev_node);
        e->setVertex(1, cur_node);
        e->setMeasurement(t);
        graph_optimizer_.addEdge(e);
      }

      // Save original pose history
      pose_history_.push_back(current_pose);
      pose_history_stamp_.push_back(odom_msg->header.stamp.toSec());

      // Save for next iteration
      previous_pose_ = corrected_pose;
      first_message_ = false;
      ROS_INFO_STREAM("[StereoSlam:] Node " << node_data.id_.data() << " insertion succeeded");
    }
  }

  // Publish map odometry
  if (odom_pub_.getNumSubscribers() > 0)
  {
    nav_msgs::Odometry odometry_msg = *odom_msg;
    odometry_msg.header.stamp = ros::Time::now();
    odometry_msg.header.frame_id = map_frame_id_;
    odometry_msg.child_frame_id = base_link_frame_id_;
    tf::poseTFToMsg(corrected_pose, odometry_msg.pose.pose);
    odom_pub_.publish(odometry_msg);
  }
}

/** \brief Timer callback. This function is called when update timer time is ellapsed.
  * @return 
  * \param event is the timer event object
  */
void stereo_slam::StereoSlamBase::timerCallback(const ros::WallTimerEvent& event)
{
  // Check if callback is currently executed
  if (block_update_)
    return;

  // Prevent for callback re-called
  block_update_ = true;

  // True when new edges to force optimization
  bool edge_added = false;

  // Find possible candidates for loop-closing
  unsigned int graph_size = graph_optimizer_.vertices().size();
  for (unsigned int i=0; i<graph_size; i++)
  {
    // Extract the pose of node i
    g2o::VertexSE3* v_i = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[i]);
    tf::Transform pose_i = stereo_slam::Utils::getNodePose(v_i);

    for (unsigned int j=i+2; j<graph_size; j++)
    {
      // Extract the pose of node j and compare
      g2o::VertexSE3* v_j = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[j]);
      tf::Transform pose_j = stereo_slam::Utils::getNodePose(v_j);

      // Check if there is an edge that currently join both nodes
      bool edge_found = false;
      for (g2o::OptimizableGraph::EdgeSet::iterator it=graph_optimizer_.edges().begin();
       it!=graph_optimizer_.edges().end(); it++)
      {
        g2o::EdgeSE3* e = dynamic_cast<g2o::EdgeSE3*> (*it);
        if (e)
        {
          if (  (e->vertices()[0]->id() == v_i->id() && e->vertices()[1]->id() == v_j->id()) ||
                (e->vertices()[0]->id() == v_j->id() && e->vertices()[1]->id() == v_i->id()) )
          {
            edge_found = true;
            break;
          }
        }
      }

      // Check if this have been discarted previously
      bool is_false = false;
      bool false_cand = stereo_slam::Utils::searchFalseCandidates(false_candidates_, v_i->id(), v_j->id());

      // If no edges found connecting this nodes, try to find loop closures
      double pose_diff = stereo_slam::Utils::poseDiff(pose_i, pose_j);
      if (!false_cand && !edge_found && 
          pose_diff < min_candidate_threshold_)
      {
        // Get the data of both nodes from database
        std::string where_i = "(id = " + boost::lexical_cast<std::string>(v_i->id() + 1) + ")";
        std::string where_j = "(id = " + boost::lexical_cast<std::string>(v_j->id() + 1) + ")";
        std::vector< boost::shared_ptr<stereo_slam::GraphNodes> > nodes_i;
        std::vector< boost::shared_ptr<stereo_slam::GraphNodes> > nodes_j;
        pg_db_ptr_thread_2_->getList(nodes_i, where_i);
        pg_db_ptr_thread_2_->getList(nodes_j, where_j);
        if (nodes_i.size() == 1 && nodes_j.size() == 1)
        {
          cv::Mat desc_i = cv::Mat_<std::vector<float> >();
          cv::Mat desc_j = cv::Mat_<std::vector<float> >();
          desc_i = stereo_slam::Utils::stdMatrixToCvMat(nodes_i[0]->descriptors_.data());
          desc_j = stereo_slam::Utils::stdMatrixToCvMat(nodes_j[0]->descriptors_.data());

          // Compute matchings
          std::vector<cv::DMatch> matches;
          stereo_slam::Utils::thresholdMatching(desc_i, desc_j, matches, descriptor_threshold_);

          if (stereo_vision_verbose_)
            ROS_INFO_STREAM("[StereoSlam:] Found " << matches.size() <<
               " matches between nodes " << v_i->id() << " and " << v_j->id() <<
               " (matches_threshold is: " << matches_threshold_ << ")");

          if ((int)matches.size() > matches_threshold_)
          {
            // Extract keypoints and 3d points of node i
            std::vector<cv::Point2f> keypoints_j;
            std::vector<cv::Point3f> points3d_i;
            keypoints_j = stereo_slam::Utils::stdMatrixToCvPoint2f(nodes_j[0]->keypoints_.data());
            points3d_i = stereo_slam::Utils::stdMatrixToCvPoint3f(nodes_i[0]->points3d_.data());
            std::vector<cv::Point2f> matched_keypoints;
            std::vector<cv::Point3f> matched_3d_points;
            for (size_t i = 0; i < matches.size(); ++i)
            {
              int index_left = matches[i].queryIdx;
              int index_right = matches[i].trainIdx;;
              matched_3d_points.push_back(points3d_i[index_left]);
              matched_keypoints.push_back(keypoints_j[index_right]);
            }
            
            // Compute the transformation between the nodes
            cv::Mat rvec, tvec;
            std::vector<int> inliers;
            cv::solvePnPRansac(matched_3d_points, matched_keypoints, camera_matrix_, 
                               cv::Mat(), rvec, tvec, false, 
                               max_solvepnp_iter_, allowed_reprojection_error_, 
                               max_inliers_, inliers);

            if (stereo_vision_verbose_)
              ROS_INFO_STREAM("[StereoSlam:] Found " << inliers.size() <<
               " inliers between nodes " << v_i->id() << " and " << v_j->id() <<
               " (min_inliers is: " << min_inliers_ << ")");

            if (static_cast<int>(inliers.size()) >= min_inliers_)
            {
              // Good! Loop closure, get the transformation matrix
              tf::Transform cl_edge = stereo_slam::Utils::buildTransformation(rvec, tvec);

              // To prevent for possible errors, compare previous transform with the new edge found
              Eigen::Isometry3d t = v_i->estimate().inverse() * v_j->estimate();
              tf::Transform cl_edge_prev = stereo_slam::Utils::eigenToTf(t);

              if (stereo_slam::Utils::poseDiff(cl_edge, cl_edge_prev) < max_edge_error_)
              {
                // Add the new edge to graph
                g2o::EdgeSE3* e = new g2o::EdgeSE3();
                Eigen::Isometry3d t = stereo_slam::Utils::tfToEigen(cl_edge);
                e->setVertex(0, v_j);
                e->setVertex(1, v_i);
                e->setMeasurement(t);
                graph_optimizer_.addEdge(e);
                edge_added = true;

                ROS_INFO_STREAM("[StereoSlam:] Loop closed between nodes " << v_i->id() << " and " << v_j->id());
              }
            }
            else
            {
              is_false = true;
            }
          }
          else
          {
            is_false = true;
          }
        }
        else
        {
          is_false = true;
        }
      }
      else
      {
        is_false = true;
      }

      // Bad candidate, save to prevent future processes
      if (is_false && !false_cand)
      {
        cv::Point2i nodes(v_i->id(), v_j->id());
        false_candidates_.push_back(nodes);
      }
    }
  }

  if (edge_added)
  {
    ROS_INFO_STREAM("[StereoSlam:] Optimizing global pose graph with " << graph_optimizer_.vertices().size() << " nodes...");
    graph_optimizer_.initializeOptimization();
    last_node_optimized_ = graph_optimizer_.vertices().size() - 1;
    graph_optimizer_.optimize(go2_opt_max_iter_);
    ROS_INFO("[StereoSlam:] Optimization done.");

    // Save graph as odometry measurments in file?
    if (save_graph_to_file_)
      saveGraph();
  }

  block_update_ = false;
}

/** \brief Reads the stereo slam node parameters
  * @return
  */
void stereo_slam::StereoSlamBase::readParameters()
{
  // Database parameters
  nh_private_.param<std::string>("db_host", db_host_, "localhost");
  nh_private_.param<std::string>("db_port", db_port_, "5432");
  nh_private_.param<std::string>("db_user", db_user_, "postgres");
  nh_private_.param<std::string>("db_pass", db_pass_, "postgres");
  nh_private_.param<std::string>("db_name", db_name_, "graph");

  // Functional parameters
  nh_private_.param("update_rate", update_rate_, 0.5);
  nh_private_.param("min_displacement", min_displacement_, 0.5);
  nh_private_.param("min_candidate_threshold", min_candidate_threshold_, 0.51);
  nh_private_.param("descriptor_threshold", descriptor_threshold_, 0.8);
  nh_private_.param<std::string>("descriptor_type", descriptor_type_, "SIFT");
  nh_private_.param("matches_threshold", matches_threshold_, 70);
  nh_private_.param("min_inliers", min_inliers_, 10);
  nh_private_.param("max_inliers", max_inliers_, 2000);
  nh_private_.param("max_solvepnp_iter", max_solvepnp_iter_, 1000);
  nh_private_.param("allowed_reprojection_error", allowed_reprojection_error_, 5.0);
  nh_private_.param("max_edge_error", max_edge_error_, 100.0);
  nh_private_.param("stereo_vision_verbose", stereo_vision_verbose_, false);

  // G2O parameters
  nh_private_.param("g2o_algorithm", g2o_algorithm_, 0);
  nh_private_.param("go2_verbose", go2_verbose_, false);
  nh_private_.param("go2_opt_max_iter", go2_opt_max_iter_, 10);

  // Topic parameters
  std::string odom_topic, left_topic, right_topic, left_info_topic, right_info_topic;
  nh_private_.param("queue_size", queue_size_, 5);
  nh_private_.param("odom_topic", odom_topic, std::string("/odometry"));
  nh_private_.param("left_topic", left_topic, std::string("/left/image_rect_color"));
  nh_private_.param("right_topic", right_topic, std::string("/right/image_rect_color"));
  nh_private_.param("left_info_topic", left_info_topic, std::string("/left/camera_info"));
  nh_private_.param("right_info_topic", right_info_topic, std::string("/right/camera_info"));
  nh_private_.param("map_frame_id", map_frame_id_, std::string("/map"));
  nh_private_.param("base_link_frame_id", base_link_frame_id_, std::string("/base_link"));

  // Graph to file parameters
  nh_private_.param("save_graph_to_file", save_graph_to_file_, false);
  nh_private_.param("files_path", files_path_, std::string("graph_data.txt"));

  // Topics subscriptions
  image_transport::ImageTransport it(nh_);
  odom_sub_ .subscribe(nh_, odom_topic, 1);
  left_sub_ .subscribe(it, left_topic, 1);
  right_sub_.subscribe(it, right_topic, 1);
  left_info_sub_.subscribe(nh_, left_info_topic, 1);
  right_info_sub_.subscribe(nh_, right_info_topic, 1);
}

/** \brief Initializates the stereo slam node
  * @return
  */
bool stereo_slam::StereoSlamBase::initializeStereoSlam()
{
  // Operational initializations
  first_message_ = true;
  first_node_ = true;
  block_update_ = false;
  last_node_optimized_ = -1;

  // Callback syncronization
  bool approx;
  nh_private_.param("approximate_sync", approx, false);
  if (approx)
  {
    approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size_),
                                    odom_sub_, 
                                    left_sub_, 
                                    right_sub_, 
                                    left_info_sub_, 
                                    right_info_sub_) );
    approximate_sync_->registerCallback(boost::bind(
        &stereo_slam::StereoSlamBase::msgsCallback,
        this, _1, _2, _3, _4, _5));
  }
  else
  {
    exact_sync_.reset(new ExactSync(ExactPolicy(queue_size_),
                                    odom_sub_, 
                                    left_sub_, 
                                    right_sub_, 
                                    left_info_sub_, 
                                    right_info_sub_) );
    exact_sync_->registerCallback(boost::bind(
        &stereo_slam::StereoSlamBase::msgsCallback, 
        this, _1, _2, _3, _4, _5));
  }

  // Advertise topics and services
  odom_pub_ = nh_private_.advertise<nav_msgs::Odometry>("odometry", 1);

  // Initialize the g2o graph optimizer
  if (g2o_algorithm_ == 0)
  {
    // Slam linear solver with gauss-newton
    SlamLinearSolver* linear_solver_ptr = new SlamLinearSolver();
    linear_solver_ptr->setBlockOrdering(false);
    SlamBlockSolver* block_solver_ptr = new SlamBlockSolver(linear_solver_ptr);
    g2o::OptimizationAlgorithmGaussNewton* solver_gauss_ptr = 
      new g2o::OptimizationAlgorithmGaussNewton(block_solver_ptr);
    graph_optimizer_.setAlgorithm(solver_gauss_ptr);
  }
  else if (g2o_algorithm_ == 1)
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
  graph_optimizer_.setVerbose(go2_verbose_);  

  // Database initialization
  boost::shared_ptr<database_interface::PostgresqlDatabase> db_ptr_1( 
    new database_interface::PostgresqlDatabase(db_host_, db_port_, db_user_, db_pass_, db_name_));
  boost::shared_ptr<database_interface::PostgresqlDatabase> db_ptr_2( 
    new database_interface::PostgresqlDatabase(db_host_, db_port_, db_user_, db_pass_, db_name_));
  pg_db_ptr_thread_1_ = db_ptr_1;
  pg_db_ptr_thread_2_ = db_ptr_2;

  if (!pg_db_ptr_thread_1_->isConnected())
  {
    ROS_ERROR("[StereoSlam:] Database failed to connect");
  }
  else
  {
    ROS_INFO("[StereoSlam:] Database connected successfully!");

    // Database table creation. New connection is needed due to the interface design
    std::string conn_info = "host=" + db_host_ + " port=" + db_port_ + 
      " user=" + db_user_ + " password=" + db_pass_ + " dbname=" + db_name_;
    connection_init_= PQconnectdb(conn_info.c_str());
    if (PQstatus(connection_init_)!=CONNECTION_OK) 
    {
      ROS_ERROR("Database connection failed with error message: %s", PQerrorMessage(connection_init_));
      return false;
    }
    else
    {
      // Drop the table (to start clean)
      std::string query_delete("DROP TABLE IF EXISTS graph_nodes");
      PQexec(connection_init_, query_delete.c_str());
      ROS_INFO("[StereoSlam:] graph_nodes table droped successfully!");

      // Create the table (if no exists)
      std::string query_create("CREATE TABLE IF NOT EXISTS graph_nodes"
                        "( "
                          "id bigserial primary key, "
                          "keypoints double precision[][], "
                          "descriptors double precision[][], "
                          "points3d double precision[][] "
                        ")");
      PQexec(connection_init_, query_create.c_str());
      ROS_INFO("[StereoSlam:] graph_nodes table created successfully!");
    }
  }

  // Start timer for graph update
  timer_ = nh_.createWallTimer(ros::WallDuration(update_rate_), 
                            &stereo_slam::StereoSlamBase::timerCallback,
                            this);

  // Parameters check
  if (matches_threshold_ < 5)
  {
    ROS_WARN("[StereoSlam:] Parameter 'matches_threshold' must be greater than 5. Set to 6.");
    matches_threshold_ = 6;
    return false;
  }
  if (files_path_[files_path_.length()-1] != '/')
    files_path_ += "/";

  return true;
}

/** \brief Save the optimized graph into a file with the same format than odometry_msgs.
  * It deletes all the file contents every time and re-write it with the last optimized.
  * @return
  */
bool stereo_slam::StereoSlamBase::saveGraph()
{
  std::string vertices_file, edges_file;
  vertices_file = files_path_ + "graph_vertices.txt";
  edges_file = files_path_ + "graph_edges.txt";

  // Open to append
  std::fstream f_vertices(vertices_file.c_str(), std::ios::out | std::ios::trunc);
  std::fstream f_edges(edges_file.c_str(), std::ios::out | std::ios::trunc);
  
  // Output the vertices file
  for (unsigned int i=0; i<graph_optimizer_.vertices().size(); i++)
  {
    // Compute timestamp
    double timestamp = pose_history_stamp_.at(i);

    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[i]);
    tf::Transform pose = stereo_slam::Utils::getNodePose(v);
    f_vertices <<  std::setprecision(19) << 
          timestamp  << "," << 
          i << "," << 
          timestamp << "," << 
          map_frame_id_ << "," << 
          base_link_frame_id_ << "," << 
          std::setprecision(6) << 
          pose.getOrigin().x() << "," << 
          pose.getOrigin().y() << "," << 
          pose.getOrigin().z() << "," << 
          pose.getRotation().x() << "," << 
          pose.getRotation().y() << "," << 
          pose.getRotation().z() << "," << 
          pose.getRotation().w() <<  std::endl;
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
        tf::Transform pose_0 = stereo_slam::Utils::getNodePose(v_0);
        tf::Transform pose_1 = stereo_slam::Utils::getNodePose(v_1);

        f_edges << counter << "," << 
              std::setprecision(6) << 
              pose_0.getOrigin().x() << "," << 
              pose_0.getOrigin().y() << "," << 
              pose_0.getOrigin().z() << "," << 
              pose_1.getOrigin().x() << "," << 
              pose_1.getOrigin().y() << "," << 
              pose_1.getOrigin().z() <<  std::endl;
        counter++;
      }
    }
  }
  f_edges.close();

  return true;
}