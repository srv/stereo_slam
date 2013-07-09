#include "stereo_slam_base.h"
#include <boost/shared_ptr.hpp>
#include <libpq-fe.h>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "postgresql_interface.h"
#include "utils.h"

/** \brief Parameter constructor. Sets the parameter struct to default values.
  */
stereo_slam::StereoSlamBase::Params::Params() : 
  db_host("localhost"),
  db_port("5432"),
  db_user("postgres"),
  db_pass("postgres"),
  db_name("graph"),  
  update_rate(DEFAULT_UPDATE_RATE),
  g2o_algorithm(DEFAULT_G2O_ALGORITHM),
  go2_opt_max_iter(DEFAULT_G2O_OPT_MAX_ITER),
  go2_verbose(DEFAULT_G2O_VERBOSE),
  min_displacement(DEFAULT_MIN_DISPLACEMENT),
  max_candidate_threshold(DEFAULT_MAX_CANDIDATE_THRESHOLD),
  neighbor_offset(DEFAULT_NEIGHBOR_OFFSET),
  descriptor_threshold(DEFAULT_DESCRIPTOR_THRESHOLD),
  epipolar_threshold(DEFAULT_EPIPOLAR_THRESHOLD),
  matches_threshold(DEFAULT_MATCHES_THRESHOLD),
  min_inliers(DEFAULT_MIN_INLIERS),
  max_inliers(DEFAULT_MAX_INLIERS),
  max_solvepnp_iter(DEFAULT_MAX_SOLVEPNP_ITER),
  allowed_reprojection_err(DEFAULT_ALLOWED_REPROJECTION_ERR),
  max_edge_err(DEFAULT_MAX_EDGE_ERR),
  stereo_vision_verbose(DEFAULT_STEREO_VISION_VERBOSE),
  bucket_width(DEFAULT_BUCKET_WIDTH),
  bucket_height(DEFAULT_BUCKET_HEIGHT),
  max_bucket_features(DEFAULT_MAX_BUCKET_FEATURES),
  queue_size(DEFAULT_QUEUE_SIZE),
  map_frame_id("/map"),
  base_link_frame_id("/base_link"),
  save_graph_to_file(DEFAULT_SAVE_GRAPH_TO_FILE),
  files_path("/home")
{}

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
  if (pose_history_.size() > 0 && last_vertex_optimized_ >= 0)
  {
    // Compute the tf between last original pose before optimization and current.
    tf::Transform last_original_pose = pose_history_.at(last_vertex_optimized_);
    tf::Transform diff = last_original_pose.inverse() * current_pose;

    // Get the last optimized pose
    g2o::VertexSE3* last_vertex =  dynamic_cast<g2o::VertexSE3*>
          (graph_optimizer_.vertices()[last_vertex_optimized_]);
    tf::Transform last_optimized_pose = stereo_slam::Utils::getVertexPose(last_vertex);

    // Compute the corrected pose
    corrected_pose = last_optimized_pose * diff;
  }

  // Check if difference between images is larger than minimum displacement
  if (stereo_slam::Utils::poseDiff(corrected_pose, previous_pose_) > params_.min_displacement
      || first_message_)
  {   
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

    // Insert this vertex into the graph database
    vertexInsertion(l_ptr, r_ptr, current_pose, corrected_pose, odom_msg->header.stamp.toSec());
  }

  // Publish slam (map)
  if (odom_pub_.getNumSubscribers() > 0)
  {
    nav_msgs::Odometry odometry_msg = *odom_msg;
    odometry_msg.header.stamp = odom_msg->header.stamp;
    odometry_msg.header.frame_id = params_.map_frame_id;
    odometry_msg.child_frame_id = params_.base_link_frame_id;
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

  // Update the graph and optimize it if new vertices have been inserted
  if (graphUpdater())
  {
    ROS_INFO_STREAM("[StereoSlam:] Optimizing global pose graph with " << graph_optimizer_.vertices().size() << " vertices...");
    graph_optimizer_.initializeOptimization();
    last_vertex_optimized_ = graph_optimizer_.vertices().size() - 1;
    graph_optimizer_.optimize(params_.go2_opt_max_iter);
    ROS_INFO("[StereoSlam:] Optimization done.");
  }

  // Save graph as odometry measurments in file
  if (params_.save_graph_to_file)
    saveGraph();

  block_update_ = false;
}

/** \brief Reads the stereo slam node parameters
  * @return
  */
void stereo_slam::StereoSlamBase::readParameters()
{
  Params stereo_slam_params;

  // Database parameters
  nh_private_.getParam("db_host", stereo_slam_params.db_host);
  nh_private_.getParam("db_port", stereo_slam_params.db_port);
  nh_private_.getParam("db_user", stereo_slam_params.db_user);
  nh_private_.getParam("db_pass", stereo_slam_params.db_pass);
  nh_private_.getParam("db_name", stereo_slam_params.db_name);

  // G2O parameters
  nh_private_.getParam("update_rate", stereo_slam_params.update_rate);
  nh_private_.getParam("g2o_algorithm", stereo_slam_params.g2o_algorithm);
  nh_private_.getParam("go2_opt_max_iter", stereo_slam_params.go2_opt_max_iter);
  nh_private_.getParam("go2_verbose", stereo_slam_params.go2_verbose);

  // Odometry operational parameters
  nh_private_.getParam("min_displacement", stereo_slam_params.min_displacement);
  nh_private_.getParam("max_candidate_threshold", stereo_slam_params.max_candidate_threshold);
  nh_private_.getParam("neighbor_offset", stereo_slam_params.neighbor_offset);

  // Stereo vision parameters
  nh_private_.getParam("descriptor_threshold", stereo_slam_params.descriptor_threshold);
  nh_private_.getParam("epipolar_threshold", stereo_slam_params.epipolar_threshold);
  nh_private_.getParam("matches_threshold", stereo_slam_params.matches_threshold);
  nh_private_.getParam("min_inliers", stereo_slam_params.min_inliers);
  nh_private_.getParam("max_inliers", stereo_slam_params.max_inliers);
  nh_private_.getParam("max_solvepnp_iter", stereo_slam_params.max_solvepnp_iter);
  nh_private_.getParam("allowed_reprojection_err", stereo_slam_params.allowed_reprojection_err);
  nh_private_.getParam("max_edge_err", stereo_slam_params.max_edge_err);
  nh_private_.getParam("stereo_vision_verbose", stereo_slam_params.stereo_vision_verbose);
  nh_private_.getParam("bucket_width", stereo_slam_params.bucket_width);
  nh_private_.getParam("bucket_height", stereo_slam_params.bucket_height);
  nh_private_.getParam("max_bucket_features", stereo_slam_params.max_bucket_features);

  // Topic parameters
  nh_private_.getParam("queue_size", stereo_slam_params.queue_size);
  nh_private_.getParam("map_frame_id", stereo_slam_params.map_frame_id);
  nh_private_.getParam("base_link_frame_id", stereo_slam_params.base_link_frame_id);

  // Graph to file parameters
  nh_private_.getParam("save_graph_to_file", stereo_slam_params.save_graph_to_file);
  nh_private_.getParam("files_path", stereo_slam_params.files_path);
  setParams(stereo_slam_params);

  // Topics subscriptions
  std::string odom_topic, left_topic, right_topic, left_info_topic, right_info_topic;
  nh_private_.param("odom_topic", odom_topic, std::string("/odometry"));
  nh_private_.param("left_topic", left_topic, std::string("/left/image_rect_color"));
  nh_private_.param("right_topic", right_topic, std::string("/right/image_rect_color"));
  nh_private_.param("left_info_topic", left_info_topic, std::string("/left/camera_info"));
  nh_private_.param("right_info_topic", right_info_topic, std::string("/right/camera_info"));
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
  first_vertex_ = true;
  block_update_ = false;
  last_vertex_optimized_ = -1;

  // Callback syncronization
  bool approx;
  nh_private_.param("approximate_sync", approx, false);
  if (approx)
  {
    approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(params_.queue_size),
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
    exact_sync_.reset(new ExactSync(ExactPolicy(params_.queue_size),
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
  graph_optimizer_.setVerbose(params_.go2_verbose);  

  // Database initialization
  boost::shared_ptr<database_interface::PostgresqlDatabase> db_ptr_1( 
    new database_interface::PostgresqlDatabase(params_.db_host, params_.db_port, params_.db_user, params_.db_pass, params_.db_name));
  boost::shared_ptr<database_interface::PostgresqlDatabase> db_ptr_2( 
    new database_interface::PostgresqlDatabase(params_.db_host, params_.db_port, params_.db_user, params_.db_pass, params_.db_name));
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
    std::string conn_info = "host=" + params_.db_host + " port=" + params_.db_port + 
      " user=" + params_.db_user + " password=" + params_.db_pass + " dbname=" + params_.db_name;
    connection_init_= PQconnectdb(conn_info.c_str());
    if (PQstatus(connection_init_)!=CONNECTION_OK) 
    {
      ROS_ERROR("[StereoSlam:] Database connection failed with error message: %s", PQerrorMessage(connection_init_));
      return false;
    }
    else
    {
      // Drop the table (to start clean)
      std::string query_delete("DROP TABLE IF EXISTS graph");
      PQexec(connection_init_, query_delete.c_str());
      ROS_INFO("[StereoSlam:] graph table dropped successfully!");

      // Create the table (if no exists)
      std::string query_create("CREATE TABLE IF NOT EXISTS graph"
                        "( "
                          "id bigserial primary key, "
                          "keypoints double precision[][], "
                          "descriptors double precision[][], "
                          "points3d double precision[][] "
                        ")");
      PQexec(connection_init_, query_create.c_str());
      ROS_INFO("[StereoSlam:] graph table created successfully!");
    }
  }

  // Start timer for graph update
  timer_ = nh_.createWallTimer(ros::WallDuration(params_.update_rate), 
                               &stereo_slam::StereoSlamBase::timerCallback,
                               this);

  // Parameters check
  if (params_.matches_threshold < 5)
  {
    ROS_WARN("[StereoSlam:] Parameter 'matches_threshold' must be greater than 5. Set to 6.");
    params_.matches_threshold = 6;
    return false;
  }
  if (params_.files_path[params_.files_path.length()-1] != '/')
    params_.files_path += "/";

  return true;
}

/** \brief Save the optimized graph into a file with the same format than odometry_msgs.
  * It deletes all the file contents every time and re-write it with the last optimized.
  * @return
  */
bool stereo_slam::StereoSlamBase::saveGraph()
{
  std::string block_file, vertices_file, edges_file;
  vertices_file = params_.files_path + "graph_vertices.txt";
  edges_file = params_.files_path + "graph_edges.txt";
  block_file = params_.files_path + ".block.txt";

  // Create a blocking element
  std::fstream f_block(block_file.c_str(), std::ios::out | std::ios::trunc);

  // Open to append
  std::fstream f_vertices(vertices_file.c_str(), std::ios::out | std::ios::trunc);
  std::fstream f_edges(edges_file.c_str(), std::ios::out | std::ios::trunc);
  
  // Output the vertices file
  for (unsigned int i=0; i<graph_optimizer_.vertices().size(); i++)
  {
    // Compute timestamp
    double timestamp = pose_history_stamp_.at(i);

    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[i]);
    tf::Transform pose = stereo_slam::Utils::getVertexPose(v);
    f_vertices <<  std::setprecision(19) << 
          timestamp  << "," << 
          i << "," << 
          timestamp << "," << 
          params_.map_frame_id << "," << 
          params_.base_link_frame_id << "," << 
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
        tf::Transform pose_0 = stereo_slam::Utils::getVertexPose(v_0);
        tf::Transform pose_1 = stereo_slam::Utils::getVertexPose(v_1);

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

  // Un-block
  f_block.close();
  int ret_code = std::remove(block_file.c_str());
  if (ret_code != 0)
    ROS_ERROR("[StereoSlam:] Error deleting the blocking file.");   

  return true;
}