#include "slam/base.h"
#include "opencv2/core/core.hpp"
#include <boost/filesystem.hpp>

namespace fs=boost::filesystem;

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
  * \param cloud_msg ros pointcloud message of type sensor_msgs::PointCloud2
  */
void slam::SlamBase::msgsCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,
                                  const sensor_msgs::ImageConstPtr& l_img_msg,
                                  const sensor_msgs::ImageConstPtr& r_img_msg,
                                  const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                  const sensor_msgs::CameraInfoConstPtr& r_info_msg,
                                  const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Get the cloud
  PointCloud::Ptr pcl_cloud(new PointCloud);
  pcl::fromROSMsg(*cloud_msg, *pcl_cloud);
  pcl::copyPointCloud(*pcl_cloud, pcl_cloud_);

  // Call the general slam callback
  msgsCallback(odom_msg, l_img_msg, r_img_msg, l_info_msg, r_info_msg);
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
  // Get the messages
  Mat l_img, r_img;
  Tools::imgMsgToMat(*l_img_msg, *r_img_msg, l_img, r_img);
  double timestamp = odom_msg->header.stamp.toSec();
  tf::Transform current_odom = Tools::odomTotf(*odom_msg);

  // Initialization
  if (first_iter_)
  {
    // Set the camera model (only once)
    Mat camera_matrix;
    image_geometry::StereoCameraModel stereo_camera_model;
    Tools::getCameraModel(*l_info_msg, *r_info_msg, stereo_camera_model, camera_matrix);
    lc_.setCameraModel(stereo_camera_model, camera_matrix);

    // Save the first node
    int cur_id = graph_.addVertice(current_odom, current_odom, timestamp);
    string lc_ref = boost::lexical_cast<string>(cur_id);
    lc_.setNode(l_img, r_img, lc_ref);

    // Publish and exit
    pose_.publish(*odom_msg, current_odom, true);
    first_iter_ = false;
    return;
  }

  // Correct the current odometry with the graph information
  tf::Transform last_graph_pose, last_graph_odom;
  graph_.getLastPoses(current_odom, last_graph_pose, last_graph_odom);
  tf::Transform corrected_odom = pose_.correctOdom(current_odom, last_graph_pose, last_graph_odom);

   // Check if difference between poses is larger than minimum displacement
  double pose_diff = Tools::poseDiff(last_graph_odom, current_odom);
  if (pose_diff <= params_.min_displacement)
  {
    pose_.publish(*odom_msg, corrected_odom);
    return;
  }

  // Insert this new node into the graph
  int cur_id = graph_.addVertice(current_odom, corrected_odom, timestamp);
  string lc_ref = boost::lexical_cast<string>(cur_id);
  lc_.setNode(l_img, r_img, lc_ref);
  ROS_INFO_STREAM("[StereoSlam:] Node " << cur_id << " inserted.");

  // Save the pointcloud for this new node
  if (params_.save_clouds && pcl_cloud_.size() > 0)
  {
    // The file name will be the timestamp
    stringstream s;
    s << fixed << setprecision(9) << timestamp;
    string filename = s.str();
    filename.erase(std::remove(filename.begin(), filename.end(), '.'), filename.end());
    pcl::io::savePCDFileBinary(params_.clouds_dir + filename + ".pcd", pcl_cloud_);
  }

  // Detect loop closures between nodes by distance
  bool any_loop_closure = false;
  vector<int> neighbors;
  graph_.findClosestNodes(params_.min_neighbour, 2, neighbors);
  for (uint i=0; i<neighbors.size(); i++)
  {
    tf::Transform edge;
    string lc_id = boost::lexical_cast<string>(neighbors[i]);
    bool valid_lc = lc_.getLoopClosureById(lc_ref, lc_id, edge);
    if (valid_lc)
    {
      ROS_INFO_STREAM("[StereoSlam:] Node with id " << cur_id << " closes loop with " << lc_id);
      graph_.addEdge(boost::lexical_cast<int>(lc_id), cur_id, edge);
      any_loop_closure = true;
    }
  }

  // TODO: do not repeat loop closures in libhaloc!

  // Detect loop closures between nodes using hashes
  int lc_id_num = -1;
  string lc_id_name = "";
  tf::Transform edge;
  bool valid_lc = lc_.getLoopClosure(lc_id_num, lc_id_name, edge);
  if (valid_lc)
  {
    ROS_INFO_STREAM("[StereoSlam:] Node with id " << cur_id << " closes loop with " << lc_id_name);
    graph_.addEdge(boost::lexical_cast<int>(lc_id_name), cur_id, edge);
    any_loop_closure = true;
  }

  // Update the graph if any loop closing
  if (any_loop_closure) graph_.update();

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
  Params params;
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
  params.clouds_dir = work_dir + "clouds/";
  if (fs::is_directory(params.clouds_dir))
    fs::remove_all(params.clouds_dir);
  fs::path dir(params.clouds_dir);
  if (!fs::create_directory(dir))
    ROS_ERROR("[StereoSlam:] ERROR -> Impossible to create the clouds directory.");

  // Topic parameters
  string odom_topic, left_topic, right_topic, left_info_topic, right_info_topic, cloud_topic;
  nh_private_.param("pose_frame_id",        pose_params.pose_frame_id,        string("/map"));
  nh_private_.param("pose_child_frame_id",  pose_params.pose_child_frame_id,  string("/robot"));
  nh_private_.param("odom_topic",           odom_topic,                       string("/odometry"));
  nh_private_.param("left_topic",           left_topic,                       string("/left/image_rect_color"));
  nh_private_.param("right_topic",          right_topic,                      string("/right/image_rect_color"));
  nh_private_.param("left_info_topic",      left_info_topic,                  string("/left/camera_info"));
  nh_private_.param("right_info_topic",     right_info_topic,                 string("/right/camera_info"));
  nh_private_.param("cloud_topic",          cloud_topic,                      string("/points2"));

  // Motion parameters
  nh_private_.param("save_clouds",  params.save_clouds, false);
  nh_private_.getParam("min_displacement",  params.min_displacement);

  // Loop closure parameters
  nh_private_.param("desc_type",            lc_params.desc_type,              string("SIFT"));
  nh_private_.param("desc_matching_type",   lc_params.desc_matching_type,     string("CROSSCHECK"));
  nh_private_.getParam("desc_thresh_ratio", lc_params.desc_thresh_ratio);
  nh_private_.getParam("min_neighbour",     lc_params.min_neighbour);
  nh_private_.getParam("n_candidates",      lc_params.n_candidates);
  nh_private_.getParam("min_matches",       lc_params.min_matches);
  nh_private_.getParam("min_inliers",       lc_params.min_inliers);

  // G2O parameters
  nh_private_.getParam("g2o_algorithm",     graph_params.g2o_algorithm);
  nh_private_.getParam("g2o_opt_max_iter",  graph_params.go2_opt_max_iter);
  graph_params.save_dir = lc_params.work_dir;
  graph_params.pose_frame_id = pose_params.pose_frame_id;
  graph_params.pose_child_frame_id = pose_params.pose_child_frame_id;

  // Set the class parameters
  params.min_neighbour = lc_params.min_neighbour;
  setParams(params);
  pose_.setParams(pose_params);
  graph_.setParams(graph_params);
  lc_.setParams(lc_params);

  // Topics subscriptions
  image_transport::ImageTransport it(nh_);
  odom_sub_       .subscribe(nh_, odom_topic,       1);
  left_sub_       .subscribe(it,  left_topic,       1);
  right_sub_      .subscribe(it,  right_topic,      1);
  left_info_sub_  .subscribe(nh_, left_info_topic,  1);
  right_info_sub_ .subscribe(nh_, right_info_topic, 1);

  if (params_.save_clouds)
    cloud_sub_    .subscribe(nh_, cloud_topic,      1);
}

/** \brief Initializes the stereo slam node
  * @return true if init OK
  */
void slam::SlamBase::init()
{
  first_iter_ = true;

  // Init Haloc
  lc_.init();

  // Advertise the pose message
  pose_.advertisePoseMsg(nh_private_);

  // Callback synchronization
  if (params_.save_clouds)
  {
    exact_sync_cloud_.reset(new ExactSyncCloud(ExactPolicyCloud(1),
                                    odom_sub_,
                                    left_sub_,
                                    right_sub_,
                                    left_info_sub_,
                                    right_info_sub_,
                                    cloud_sub_) );
    exact_sync_cloud_->registerCallback(boost::bind(
        &slam::SlamBase::msgsCallback,
        this, _1, _2, _3, _4, _5, _6));
  }
  else
  {
    exact_sync_no_cloud_.reset(new ExactSyncNoCloud(ExactPolicyNoCloud(1),
                                    odom_sub_,
                                    left_sub_,
                                    right_sub_,
                                    left_info_sub_,
                                    right_info_sub_) );
    exact_sync_no_cloud_->registerCallback(boost::bind(
        &slam::SlamBase::msgsCallback,
        this, _1, _2, _3, _4, _5));
  }

}