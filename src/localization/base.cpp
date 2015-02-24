#include "localization/base.h"
#include "opencv2/core/core.hpp"
#include <boost/filesystem.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>


using namespace boost;
namespace fs=filesystem;


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


/** \brief Generic odometry callback. This function is called when an odometry message is received
* @return
* \param odom_msg ros odometry message of type nav_msgs::Odometry
*/
void slam::SlamBase::genericCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  if (!params_.enable)
  {
    // The slam is not enabled, re-publish the input odometry

    // Get the current odometry
    tf::Transform current_odom_robot = Tools::odomTotf(*odom_msg);

    // Publish
    publish(*odom_msg, current_odom_robot);
  }
  else
  {
    // Check if syncronized callback is working, i.e. corrected odometry is published regularly
    ros::WallDuration elapsed_time = ros::WallTime::now() - last_pub_odom_;
    if (elapsed_time.toSec() < 2.0)
    {
      // It seems the msgCallback is publishing corrected odometry regularly ;)
      return;
    }

    ROS_WARN_STREAM("[Localization:] We are not getting synchronized messages regularly. No SLAM corrections will be performed. Elapsed time: " << elapsed_time.toSec());

    // Get the current odometry
    tf::Transform current_odom_robot = Tools::odomTotf(*odom_msg);

    // Have we a corrected pose? Is the graph initialized?
    if (graph_.numNodes() > 0)
    {
      // The graph is initialized, so publish a corrected odometry

      // Transform the odometry to the camera frame
      tf::Transform current_odom_camera = current_odom_robot * odom2camera_;

      // Correct the current odometry with the graph information
      tf::Transform last_graph_pose, last_graph_odom;
      graph_.getLastPoses(current_odom_camera, last_graph_pose, last_graph_odom);
      tf::Transform corrected_odom = pose_.correctPose(current_odom_camera, last_graph_pose, last_graph_odom);

      // Publish
      publish(*odom_msg, corrected_odom * odom2camera_.inverse());
    }
    else
    {
      // There is nothing to correct... Publish the robot odometry directly.
      publish(*odom_msg, current_odom_robot);
    }
  }
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
  PointCloudRGB::Ptr pcl_cloud(new PointCloudRGB);
  pcl::fromROSMsg(*cloud_msg, *pcl_cloud);
  pcl::copyPointCloud(*pcl_cloud, pcl_cloud_);

  // Run the general slam callback
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
  // Get the current timestamp
  double timestamp = l_img_msg->header.stamp.toSec();

  // Get the current odometry
  tf::Transform current_odom_robot = Tools::odomTotf(*odom_msg);

  // Get the images
  Mat l_img, r_img;
  Tools::imgMsgToMat(*l_img_msg, *r_img_msg, l_img, r_img);

  // Initialization
  if (first_iter_)
  {
    // Get the transform between odometry frame and camera frame
    if (!getOdom2CameraTf(*odom_msg, *l_img_msg, odom2camera_))
    {
      ROS_WARN("[Localization:] Impossible to transform odometry to camera frame.");
      return;
    }

    // Set the camera model (only once)
    Mat camera_matrix;
    image_geometry::StereoCameraModel stereo_camera_model;
    Tools::getCameraModel(*l_info_msg, *r_info_msg, stereo_camera_model, camera_matrix);
    lc_.setCameraModel(stereo_camera_model, camera_matrix);

    // Transform the odometry to the camera frame
    tf::Transform current_odom_camera = current_odom_robot * odom2camera_;

    // Set the first node to libhaloc
    int id_tmp = lc_.setNode(l_img, r_img);

    if (id_tmp >= 0)
    {
      // Add the vertex and save the cloud (if any)
      int cur_id = graph_.addVertex(current_odom_camera, current_odom_camera, timestamp);
      ROS_INFO_STREAM("[Localization:] Node " << cur_id << " inserted.");
      processCloud(cur_id);

      // Publish
      publish(*odom_msg, current_odom_robot);

      // Slam initialized!
      first_iter_ = false;
    }
    else
    {
      // Slam is not already initialized, publish the current odometry
      publish(*odom_msg, current_odom_robot);
    }

    // Exit
    return;
  }

  // Transform the odometry to the camera frame
  tf::Transform current_odom_camera = current_odom_robot * odom2camera_;

  // Correct the current odometry with the graph information
  tf::Transform last_graph_pose, last_graph_odom;
  graph_.getLastPoses(current_odom_camera, last_graph_pose, last_graph_odom);
  tf::Transform corrected_odom = pose_.correctPose(current_odom_camera, last_graph_pose, last_graph_odom);

   // Check if difference between poses is larger than minimum displacement
  double pose_diff = Tools::poseDiff(last_graph_odom, current_odom_camera);
  if (pose_diff <= params_.min_displacement)
  {
    // Publish and exit
    publish(*odom_msg, corrected_odom * odom2camera_.inverse());
    return;
  }

  // Insert this new node into libhaloc
  int id_tmp = lc_.setNode(l_img, r_img);

  // Check if node has been inserted
  if (id_tmp < 0)
  {
    // Publish and exit
    ROS_DEBUG("[Localization:] Impossible to save the node due to its poor quality.");
    publish(*odom_msg, corrected_odom * odom2camera_.inverse());
    return;
  }

  // Decide if the position of this node will be computed by SolvePNP or odometry
  tf::Transform corrected_pose = corrected_odom;
  if (params_.refine_neighbors)
  {
    tf::Transform vertex_disp;
    int last_id = graph_.getLastVertexId();
    bool valid = lc_.getLoopClosure(lexical_cast<string>(last_id), lexical_cast<string>(last_id+1), vertex_disp);
    if (valid)
    {
      ROS_INFO("[Localization:] Pose refined.");
      corrected_pose = last_graph_pose * vertex_disp;
    }
  }

  // Add the vertex and save the cloud (if any)
  int cur_id = graph_.addVertex(current_odom_camera, corrected_pose, timestamp);
  ROS_INFO_STREAM("[Localization:] Node " << cur_id << " inserted.");
  processCloud(cur_id);

  // Detect loop closures between nodes by distance
  bool any_loop_closure = false;
  vector<int> neighbors;
  graph_.findClosestNodes(params_.min_neighbor, 2, neighbors);
  for (uint i=0; i<neighbors.size(); i++)
  {
    tf::Transform edge;
    string lc_id = lexical_cast<string>(neighbors[i]);
    bool valid_lc = lc_.getLoopClosure(lexical_cast<string>(cur_id), lc_id, edge, true);
    if (valid_lc)
    {
      //ROS_INFO_STREAM("[Localization:] Node with id " << cur_id << " closes loop with " << lc_id);
      cout << "\033[1;32m[ INFO]: [Localization:] Node with id " << cur_id << " closes loop with " << lc_id << "\033[0m\n";
      graph_.addEdge(lexical_cast<int>(lc_id), cur_id, edge);
      any_loop_closure = true;
    }
  }

  // Detect loop closures between nodes using hashes
  int lc_id_num = -1;
  tf::Transform edge;
  bool valid_lc = lc_.getLoopClosure(lc_id_num, edge);
  if (valid_lc)
  {
    //ROS_INFO_STREAM("[Localization:] Node with id " << cur_id << " closes loop with " << lc_id_num);
    cout << "\033[1;32m[ INFO]: [Localization:] Node with id " << cur_id << " closes loop with " << lc_id_num << "\033[0m\n";
    graph_.addEdge(lc_id_num, cur_id, edge);
    any_loop_closure = true;
  }

  // Update the graph if any loop closing
  if (any_loop_closure) graph_.update();

  // Publish the slam pose
  graph_.getLastPoses(current_odom_camera, last_graph_pose, last_graph_odom);
  publish(*odom_msg, last_graph_pose * odom2camera_.inverse());

  // Save graph to file and send (if needed)
  graph_.saveToFile(odom2camera_.inverse());

  return;
}


/** \brief Filters a pointcloud
  * @return filtered cloud
  * \param input cloud
  */
PointCloudRGB::Ptr slam::SlamBase::filterCloud(PointCloudRGB::Ptr in_cloud)
{
  // Remove nans
  vector<int> indicies;
  PointCloudRGB::Ptr cloud(new PointCloudRGB);
  pcl::removeNaNFromPointCloud(*in_cloud, *cloud, indicies);

  // Limit filtering
  PointCloudRGB::Ptr cloud_filtered_ptr(new PointCloudRGB);
  pcl::PassThrough<PointRGB> pass;
  pass.setFilterFieldName("x");
  pass.setFilterLimits(params_.x_filter_min, params_.x_filter_max);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(params_.y_filter_min, params_.y_filter_max);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(params_.z_filter_min, params_.z_filter_max);
  pass.setInputCloud(cloud);
  pass.filter(*cloud_filtered_ptr);

  return cloud_filtered_ptr;
}


/** \brief Save and/or send the accumulated cloud depending on the value of 'save_clouds' and 'listen_reconstruction_srv'
 * \param Cloud id
 */
void slam::SlamBase::processCloud(int cloud_id)
{
  // Proceed?
  if (pcl_cloud_.size() == 0 || !params_.save_clouds ) return;

  // Cloud filtering
  PointCloudRGB::Ptr cloud_filtered(new PointCloudRGB);
  cloud_filtered = filterCloud(pcl_cloud_.makeShared());

  // Save cloud
  string id = lexical_cast<string>(cloud_id);
  pcl::io::savePCDFileBinary(params_.clouds_dir + id + ".pcd", pcl_cloud_);
}


/** \brief Get the transform between odometry frame and camera frame
  * @return true if valid transform, false otherwise
  * \param Odometry msg
  * \param Image msg
  * \param Output transform
  */
bool slam::SlamBase::getOdom2CameraTf(nav_msgs::Odometry odom_msg,
                                      sensor_msgs::Image img_msg,
                                      tf::StampedTransform &transform)
{
  // Init the transform
  transform.setIdentity();

  try
  {
    // Extract the transform
    tf_listener_.lookupTransform(odom_msg.child_frame_id,
                                 img_msg.header.frame_id,
                                 ros::Time(0),
                                 transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }
  return true;
}


/** \brief Publish odometry and information messages
  */
void slam::SlamBase::publish(nav_msgs::Odometry odom_msg, tf::Transform odom)
{
  // Publish the odometry message
  pose_.publish(odom_msg, odom);
  last_pub_odom_ = ros::WallTime::now();

  // Information message
  if (info_pub_.getNumSubscribers() > 0)
  {
    stereo_slam::SlamInfo info_msg;
    info_msg.num_nodes = graph_.numNodes();
    info_msg.num_loop_closures = graph_.numLoopClosures();
    info_pub_.publish(info_msg);
  }
}


/** \brief Creates the clouds directory (reset if exists)
  */
void slam::SlamBase::createCloudsDir()
{
  if (fs::is_directory(params_.clouds_dir))
    fs::remove_all(params_.clouds_dir);
  fs::path dir(params_.clouds_dir);
  if (!fs::create_directory(dir))
    ROS_ERROR("[Localization:] ERROR -> Impossible to create the clouds directory.");
}


/** \brief Reads the stereo slam node parameters
  */
void slam::SlamBase::readParameters()
{
  Params params;
  slam::Pose::Params pose_params;
  slam::Graph::Params graph_params;
  haloc::LoopClosure::Params lc_params;
  lc_params.num_proj = 2;
  lc_params.verbose = true;

  // Operational directories
  string work_dir;
  nh_private_.param("work_dir", work_dir, string(""));
  if (work_dir[work_dir.length()-1] != '/')
    work_dir += "/";
  lc_params.work_dir = work_dir;
  params.clouds_dir = work_dir + "clouds/";

  // Enable
  nh_private_.param("enable", params.enable, true);

  // Topic parameters
  string odom_topic, left_topic, right_topic, left_info_topic, right_info_topic, cloud_topic;
  nh_private_.param("pose_frame_id",              pose_params.pose_frame_id,        string("/map"));
  nh_private_.param("pose_child_frame_id",        pose_params.pose_child_frame_id,  string("/robot"));
  nh_private_.param("odom_topic",                 odom_topic,                       string("/odometry"));
  nh_private_.param("left_topic",                 left_topic,                       string("/left/image_rect_color"));
  nh_private_.param("right_topic",                right_topic,                      string("/right/image_rect_color"));
  nh_private_.param("left_info_topic",            left_info_topic,                  string("/left/camera_info"));
  nh_private_.param("right_info_topic",           right_info_topic,                 string("/right/camera_info"));
  nh_private_.param("cloud_topic",                cloud_topic,                      string("/points2"));

  // Motion parameters
  nh_private_.param("refine_neighbors",           params.refine_neighbors,          false);
  nh_private_.param("min_displacement",           params.min_displacement,          0.3);

  // 3D reconstruction parameters
  nh_private_.param("save_clouds",                params.save_clouds,               false);

  // Loop closure parameters
  nh_private_.param("desc_type",                  lc_params.desc_type,              string("SIFT"));
  nh_private_.param("desc_matching_type",         lc_params.desc_matching_type,     string("CROSSCHECK"));
  nh_private_.param("desc_thresh_ratio",          lc_params.desc_thresh_ratio,      0.8);
  nh_private_.param("min_neighbor",               lc_params.min_neighbor,           10);
  nh_private_.param("n_candidates",               lc_params.n_candidates,           5);
  nh_private_.param("min_matches",                lc_params.min_matches,            100);
  nh_private_.param("min_inliers",                lc_params.min_inliers,            50);

  // G2O parameters
  nh_private_.param("g2o_algorithm",              graph_params.g2o_algorithm,       0);
  nh_private_.param("g2o_opt_max_iter",           graph_params.go2_opt_max_iter,    20);

  // Cloud filtering values
  nh_private_.param("x_filter_min",               params.x_filter_min,              -3.0);
  nh_private_.param("x_filter_max",               params.x_filter_max,              3.0);
  nh_private_.param("y_filter_min",               params.y_filter_min,              -3.0);
  nh_private_.param("y_filter_max",               params.y_filter_max,              3.0);
  nh_private_.param("z_filter_min",               params.z_filter_min,              0.2);
  nh_private_.param("z_filter_max",               params.z_filter_max,              6.0);

  // Some other graph parameters
  graph_params.save_dir = lc_params.work_dir;
  graph_params.pose_frame_id = pose_params.pose_frame_id;
  graph_params.pose_child_frame_id = pose_params.pose_child_frame_id;

  // Set the class parameters
  params.odom_topic = odom_topic;
  params.min_neighbor = lc_params.min_neighbor;
  setParams(params);
  pose_.setParams(pose_params);
  graph_.setParams(graph_params);
  lc_.setParams(lc_params);

  // Topics subscriptions
  image_transport::ImageTransport it(nh_);
  odom_sub_       .subscribe(nh_, odom_topic,       25);
  left_sub_       .subscribe(it,  left_topic,       3);
  right_sub_      .subscribe(it,  right_topic,      3);
  left_info_sub_  .subscribe(nh_, left_info_topic,  3);
  right_info_sub_ .subscribe(nh_, right_info_topic, 3);

  if (params_.save_clouds)
    cloud_sub_.subscribe(nh_, cloud_topic, 5);
}


/** \brief Initializes the stereo slam node
  */
void slam::SlamBase::init()
{
  // Advertise the slam odometry message
  pose_.advertisePoseMsg(nh_private_);

  // Generic subscriber
  generic_sub_ = nh_.subscribe<nav_msgs::Odometry>(params_.odom_topic, 1, &SlamBase::genericCallback, this);

  // Enable the slam?
  if (params_.enable)
  {
    // Init
    first_iter_ = true;
    last_pub_odom_ = ros::WallTime::now();
    odom2camera_.setIdentity();

    // Init Haloc
    lc_.init();

    // Init Graph
    graph_.init();

    // Advertise the info message
    info_pub_ = nh_private_.advertise<stereo_slam::SlamInfo>("info", 1);

    // Cloud callback
    if (params_.save_clouds)
    {
      // Create the directory for clouds
      createCloudsDir();

      // Create the callback with the clouds
      sync_cloud_.reset(new SyncCloud(PolicyCloud(5),
                                      odom_sub_,
                                      left_sub_,
                                      right_sub_,
                                      left_info_sub_,
                                      right_info_sub_,
                                      cloud_sub_) );
      sync_cloud_->registerCallback(bind(
          &slam::SlamBase::msgsCallback,
          this, _1, _2, _3, _4, _5, _6));
    }
    else
    {
      // Callback without clouds
      sync_no_cloud_.reset(new SyncNoCloud(PolicyNoCloud(3),
                                      odom_sub_,
                                      left_sub_,
                                      right_sub_,
                                      left_info_sub_,
                                      right_info_sub_) );
      sync_no_cloud_->registerCallback(bind(
          &slam::SlamBase::msgsCallback,
          this, _1, _2, _3, _4, _5));
    }
  }
}


/** \brief Finalize stereo slam node
  * @return
  */
void slam::SlamBase::finalize()
{
  ROS_INFO("[Localization:] Finalizing...");
  lc_.finalize();
  ROS_INFO("[Localization:] Done!");
}