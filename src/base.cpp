#include "base.h"
#include "tools.h"
#include "opencv2/core/core.hpp"
#include <cv_bridge/cv_bridge.h>

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
  init();
}

/** \brief Messages callback. This function is called when syncronized odometry and image
  * message are received.
  * @return 
  * \param odom_msg ros odometry message of type nav_msgs::Odometry
  * \param l_img left stereo image message of type sensor_msgs::Image
  * \param r_img right stereo image message of type sensor_msgs::Image
  * \param l_info left stereo info message of type sensor_msgs::CameraInfo
  * \param r_info right stereo info message of type sensor_msgs::CameraInfo
  */
void stereo_slam::StereoSlamBase::msgsCallback(
                                  const nav_msgs::Odometry::ConstPtr& odom_msg,
                                  const sensor_msgs::ImageConstPtr& l_img_msg,
                                  const sensor_msgs::ImageConstPtr& r_img_msg,
                                  const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                  const sensor_msgs::CameraInfoConstPtr& r_info_msg)
{
  // Get the current odometry for these images
  tf::Transform current_odom = stereo_slam::Tools::odomTotf(*odom_msg);

  // Get the latest poses of the graph (if any)
  tf::Transform last_graph_pose, last_graph_odom;
  graph_.getLastPoses(current_odom, last_graph_pose, last_graph_odom);

  // Correct current odometry with the graph information
  tf::Transform corrected_odom = pose_.correctOdom(current_odom, last_graph_pose, last_graph_odom);

   // Check if difference between poses is larger than minimum displacement 
  double pose_diff = stereo_slam::Tools::poseDiff(last_graph_odom, current_odom);
  if (pose_diff <= params_.min_displacement && !first_iter_)
  {
    ROS_INFO_STREAM("[StereoSlam:] Small displacement (" << pose_diff << ") publishing corrected pose.");

    // Publish and exit
    pose_.publish(*odom_msg, corrected_odom);
    return;
  }

  // Insert this new node into the graph
  int cur_id = graph_.addVertice(current_odom, corrected_odom, odom_msg->header.stamp.toSec());

  // Get the images from message
  Mat l_img, r_img;
  getImages(*l_img_msg, *r_img_msg, *l_info_msg, *r_info_msg, l_img, r_img);

  // Detect loop closure
  int img_lc = -1;
  string lc_id_str = "";
  tf::Transform edge;
  lc_.setNode(l_img, r_img, boost::lexical_cast<string>(cur_id));
  if (!lc_.getLoopClosure(img_lc, lc_id_str, edge))
  {
    // No loop closures, publish the corrected pose and exit
    ROS_INFO_STREAM("[StereoSlam:] New node inserted with id " << cur_id << " (no closes loop).");
    pose_.publish(*odom_msg, corrected_odom);
    graph_.saveGraphToFile();
    return;
  }

  // Insert the new loop closure into the graph and update
  ROS_INFO_STREAM("[StereoSlam:] New node inserted with id " << cur_id << " closes loop with " << lc_id_str);
  graph_.addEdge(boost::lexical_cast<int>(lc_id_str), cur_id, edge);
  graph_.update();

  // Publish the corrected pose
  pose_.publish(*odom_msg, corrected_odom);

  // Save graph to file
  graph_.saveGraphToFile();
  return;
}


/** \brief Reads the stereo slam node parameters
  * @return
  */
void stereo_slam::StereoSlamBase::readParameters()
{
  Params stereo_slam_params;
  stereo_slam::Pose::Params pose_params;
  stereo_slam::Graph::Params graph_params;
  haloc::LoopClosure::Params lc_params;
  lc_params.num_proj = 2;
  lc_params.validate = true;

  // Operational directories
  nh_private_.param("work_dir", lc_params.work_dir, string(""));

  // Topic parameters
  string odom_topic, left_topic, right_topic, left_info_topic, right_info_topic;
  nh_private_.getParam("pose_frame_id", pose_params.pose_frame_id);
  nh_private_.getParam("pose_child_frame_id", pose_params.pose_child_frame_id);
  nh_private_.param("odom_topic", odom_topic, string("/odometry"));
  nh_private_.param("left_topic", left_topic, string("/left/image_rect_color"));
  nh_private_.param("right_topic", right_topic, string("/right/image_rect_color"));
  nh_private_.param("left_info_topic", left_info_topic, string("/left/camera_info"));
  nh_private_.param("right_info_topic", right_info_topic, string("/right/camera_info"));

  // Motion parameters
  nh_private_.getParam("min_displacement", stereo_slam_params.min_displacement);

  // Loop closure parameters
  nh_private_.param("desc_type", lc_params.desc_type, string("SIFT"));
  nh_private_.getParam("desc_thresh", lc_params.desc_thresh);
  nh_private_.getParam("epipolar_thresh", lc_params.epipolar_thresh);
  nh_private_.getParam("min_neighbour", lc_params.min_neighbour);
  nh_private_.getParam("n_candidates", lc_params.n_candidates);
  nh_private_.getParam("min_matches", lc_params.min_matches);
  nh_private_.getParam("min_inliers", lc_params.min_inliers);

  // G2O parameters
  nh_private_.getParam("g2o_algorithm", graph_params.g2o_algorithm);
  nh_private_.getParam("g2o_opt_max_iter", graph_params.go2_opt_max_iter);
  graph_params.save_dir = lc_params.work_dir;
  graph_params.pose_frame_id = pose_params.pose_frame_id;
  graph_params.pose_child_frame_id = pose_params.pose_child_frame_id;

  // Set the class parameters
  setParams(stereo_slam_params);
  pose_.setParams(pose_params);
  graph_.setParams(graph_params);
  lc_.setParams(lc_params);

  // Topics subscriptions
  image_transport::ImageTransport it(nh_);
  odom_sub_ .subscribe(nh_, odom_topic, 1);
  left_sub_ .subscribe(it, left_topic, 1);
  right_sub_.subscribe(it, right_topic, 1);
  left_info_sub_.subscribe(nh_, left_info_topic, 1);
  right_info_sub_.subscribe(nh_, right_info_topic, 1);
}

/** \brief Initializates the stereo slam node
  * @return true if init ok
  */
bool stereo_slam::StereoSlamBase::init()
{
  first_iter_ = true;

  // Init Haloc
  lc_.init();

  // Advertice the pose message
  pose_.adverticePoseMsg(nh_private_);

  // Callback syncronization
  approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(1),
                                  odom_sub_, 
                                  left_sub_, 
                                  right_sub_, 
                                  left_info_sub_, 
                                  right_info_sub_) );
  approximate_sync_->registerCallback(boost::bind(
      &stereo_slam::StereoSlamBase::msgsCallback,
      this, _1, _2, _3, _4, _5));

  return true;
}

/** \brief Get the images from the ros messages and scale it
  * \param l_img left stereo image message of type sensor_msgs::Image
  * \param r_img right stereo image message of type sensor_msgs::Image
  * \param l_info left stereo info message of type sensor_msgs::CameraInfo
  * \param r_info right stereo info message of type sensor_msgs::CameraInfo
  * \param left scaled output image
  * \param right scaled output image
  * @return true if all ok.
  */
bool stereo_slam::StereoSlamBase::getImages(sensor_msgs::Image l_img_msg, 
                                            sensor_msgs::Image r_img_msg, 
                                            sensor_msgs::CameraInfo l_info_msg, 
                                            sensor_msgs::CameraInfo r_info_msg, 
                                            Mat &l_img, Mat &r_img)
{
  // Convert message to cv::Mat
  Mat l_img_src, r_img_src;
  try
  {
    l_img_src = (cv_bridge::toCvCopy(l_img_msg, enc::BGR8))->image;
    r_img_src = (cv_bridge::toCvCopy(r_img_msg, enc::BGR8))->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("[StereoSlam:] cv_bridge exception: %s", e.what());
    return false;
  }

  // Set camera model (only once)
  if (first_iter_)
  {
    scale_factor_ = 320.0/l_img_src.cols;
    stereo_camera_model_.fromCameraInfo(l_info_msg, r_info_msg);
    const cv::Mat P(3,4, CV_64FC1, const_cast<double*>(l_info_msg.P.data()));
    camera_matrix_ = P.colRange(cv::Range(0,3)).clone();
    first_iter_ = false;
  }

  // Scale images
  resize(l_img_src, l_img, Size(), scale_factor_, scale_factor_);
  resize(r_img_src, r_img, Size(), scale_factor_, scale_factor_);

  return true;
}