#include "slam/base.h"
#include "common/tools.h"
#include "opencv2/core/core.hpp"
#include <cv_bridge/cv_bridge.h>

/** \brief Class constructor. Reads node parameters and initialize some properties.
  * @return
  * \param nh public node handler
  * \param nhp private node handler
  */
slam::SlamBase::SlamBase(
  ros::NodeHandle nh, ros::NodeHandle nhp) : nh_(nh), nh_private_(nhp)
{
  // Read the node parameters
  readParameters();

  // Initialize the stereo slam
  init();
}

/** \brief Finalize stereo slam node
  * @return
  */
void slam::SlamBase::finalize()
{
  ROS_INFO("[StereoSlam:] Finalizing...");
  lc_.finalize();
  ROS_INFO("[StereoSlam:] Done!");
}

/** \brief Messages callback. This function is called when synchronized odometry and image
  * message are received.
  * @return
  * \param odom_msg ros odometry message of type nav_msgs::Odometry
  * \param l_img left stereo image message of type sensor_msgs::Image
  * \param r_img right stereo image message of type sensor_msgs::Image
  * \param l_info left stereo info message of type sensor_msgs::CameraInfo
  * \param r_info right stereo info message of type sensor_msgs::CameraInfo
  */
void slam::SlamBase::msgsCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,
                                  const sensor_msgs::ImageConstPtr& l_img_msg,
                                  const sensor_msgs::ImageConstPtr& r_img_msg,
                                  const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                  const sensor_msgs::CameraInfoConstPtr& r_info_msg)
{
  // Get the current odometry for these images
  tf::Transform current_odom = slam::Tools::odomTotf(*odom_msg);

  // Get the latest poses of the graph (if any)
  tf::Transform last_graph_pose, last_graph_odom;
  graph_.getLastPoses(current_odom, last_graph_pose, last_graph_odom);

  // Correct current odometry with the graph information
  tf::Transform corrected_odom = pose_.correctOdom(current_odom, last_graph_pose, last_graph_odom);

   // Check if difference between poses is larger than minimum displacement
  double pose_diff = slam::Tools::poseDiff(last_graph_odom, current_odom);
  if (pose_diff <= params_.min_displacement && !first_iter_)
  {
    // Publish and exit
    pose_.publish(*odom_msg, corrected_odom, false);
    return;
  }

  // Insert this new node into the graph
  int cur_id = graph_.addVertice(current_odom, corrected_odom, odom_msg->header.stamp.toSec());

  // Get the images from message
  Mat l_img, r_img;
  getImages(*l_img_msg, *r_img_msg, *l_info_msg, *r_info_msg, l_img, r_img);

  // Detect loop closure
  int img_lc = -1;
  string lc_id = "";
  tf::Transform edge;
  lc_.setNode(l_img, r_img, boost::lexical_cast<string>(cur_id));
  if (!lc_.getLoopClosure(img_lc, lc_id, edge))
  {
    // No loop closures, publish the corrected pose and exit
    ROS_INFO_STREAM("[StereoSlam:] New node inserted with id " << cur_id << " (no closes loop).");
    pose_.publish(*odom_msg, corrected_odom, true);
    graph_.saveGraphToFile();
    return;
  }

  // Insert the new loop closure into the graph and update
  ROS_INFO_STREAM("[StereoSlam:] New node inserted with id " << cur_id << " closes loop with " << lc_id);
  graph_.addEdge(boost::lexical_cast<int>(lc_id), cur_id, edge);
  graph_.update();

  // Publish the corrected pose
  graph_.getLastPoses(current_odom, last_graph_pose, last_graph_odom);
  corrected_odom = pose_.correctOdom(current_odom, last_graph_pose, last_graph_odom);
  pose_.publish(*odom_msg, corrected_odom, true);

  // Save graph to file
  graph_.saveGraphToFile();
  return;
}


/** \brief Reads the stereo slam node parameters
  * @return
  */
void slam::SlamBase::readParameters()
{
  Params slam_params;
  slam::Pose::Params pose_params;
  slam::Graph::Params graph_params;
  haloc::LoopClosure::Params lc_params;
  lc_params.num_proj = 2;
  lc_params.validate = false;
  lc_params.verbose = true;

  // Operational directories
  string work_dir;
  nh_private_.param("work_dir", work_dir, string(""));
  if (work_dir[work_dir.length()-1] != '/')
    work_dir += "/";
  lc_params.work_dir = work_dir;

  // Topic parameters
  string odom_topic, left_topic, right_topic, left_info_topic, right_info_topic;
  nh_private_.param("pose_frame_id", pose_params.pose_frame_id, string("/map"));
  nh_private_.param("pose_child_frame_id", pose_params.pose_child_frame_id, string("/robot"));
  nh_private_.param("odom_topic", odom_topic, string("/odometry"));
  nh_private_.param("left_topic", left_topic, string("/left/image_rect_color"));
  nh_private_.param("right_topic", right_topic, string("/right/image_rect_color"));
  nh_private_.param("left_info_topic", left_info_topic, string("/left/camera_info"));
  nh_private_.param("right_info_topic", right_info_topic, string("/right/camera_info"));

  // Motion parameters
  nh_private_.getParam("min_displacement", slam_params.min_displacement);

  // Loop closure parameters
  nh_private_.param("desc_type", lc_params.desc_type, string("SIFT"));
  nh_private_.getParam("desc_thresh", lc_params.desc_thresh);
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
  setParams(slam_params);
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

/** \brief Initializes the stereo slam node
  * @return true if init OK
  */
bool slam::SlamBase::init()
{
  first_iter_ = true;

  // Init Haloc
  lc_.init();

  // Advertise the pose message
  pose_.advertisePoseMsg(nh_private_);

  // Callback synchronization
  exact_sync_.reset(new ExactSync(ExactPolicy(10),
                                  odom_sub_,
                                  left_sub_,
                                  right_sub_,
                                  left_info_sub_,
                                  right_info_sub_) );
  exact_sync_->registerCallback(boost::bind(
      &slam::SlamBase::msgsCallback,
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
  * @return true if all OK.
  */
bool slam::SlamBase::getImages(sensor_msgs::Image l_img_msg,
                                            sensor_msgs::Image r_img_msg,
                                            sensor_msgs::CameraInfo l_info_msg,
                                            sensor_msgs::CameraInfo r_info_msg,
                                            Mat &l_img, Mat &r_img)
{
  // Convert message to Mat
  try
  {
    l_img = (cv_bridge::toCvCopy(l_img_msg, enc::BGR8))->image;
    r_img = (cv_bridge::toCvCopy(r_img_msg, enc::BGR8))->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("[StereoSlam:] cv_bridge exception: %s", e.what());
    return false;
  }

  // Set camera model (only once)
  if (first_iter_)
  {
    // Get the stereo camera model
    image_geometry::StereoCameraModel stereo_camera_model;
    stereo_camera_model.fromCameraInfo(l_info_msg, r_info_msg);

    // Get the projection/camera matrix
    const Mat P(3,4, CV_64FC1, const_cast<double*>(l_info_msg.P.data()));
    Mat camera_matrix = P.colRange(Range(0,3)).clone();

    // Are the images scaled?
    int binning_x = l_info_msg.binning_x;
    int binning_y = l_info_msg.binning_y;
    if (binning_x > 1 || binning_y > 1)
    {
      camera_matrix.at<double>(0,0) = camera_matrix.at<double>(0,0) / binning_x;
      camera_matrix.at<double>(0,2) = camera_matrix.at<double>(0,2) / binning_x;
      camera_matrix.at<double>(1,1) = camera_matrix.at<double>(1,1) / binning_y;
      camera_matrix.at<double>(1,2) = camera_matrix.at<double>(1,2) / binning_y;
    }

    // Set all for the loop closure class
    lc_.setCameraModel(stereo_camera_model, camera_matrix);
    first_iter_ = false;
  }
  return true;
}