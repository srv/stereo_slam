#include <signal.h>

#include "localization/base.h"
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <boost/filesystem.hpp>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/approximate_voxel_grid.h>

using namespace boost;
namespace fs=filesystem;

// Stop handler binding
boost::function<void(int)> stopHandlerCb;

/** \brief Catches the Ctrl+C signal.
  */
void stopHandler(int s)
{
  printf("Caught signal %d\n",s);
  stopHandlerCb(s);
  ros::shutdown();
}


/** \brief Class constructor. Reads node parameters and initialize some properties.
  * @return
  * \param nh public node handler
  * \param nhp private node handler
  */
slam::SlamBase::SlamBase(
  ros::NodeHandle nh, ros::NodeHandle nhp) : nh_(nh), nh_private_(nhp)
{
  // Bind the finalize member to stopHandler signal
  stopHandlerCb = std::bind1st(std::mem_fun(&slam::SlamBase::finalize), this);

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
      corrected_odom = corrected_odom * odom2camera_.inverse();

      // Check the difference between previous and this odometry
      double pose_diff = Tools::poseDiff(last_pub_odom_, corrected_odom);
      if (pose_diff > params_.max_correction)
      {
        // Exit without publishing odometry
        ROS_ERROR_STREAM("[Localization:] The correction between previous and current odometry is too large: " << pose_diff);
        return;
      }

      // Interpolate the output (to avoid large jumps when graph corrections are applied)
      // Compute the time of the interpolation depending on the size of the odometry jump
      double time_to_interpolate = pose_diff*params_.correction_interp_time/params_.max_correction;
      double elapsed_time = ros::WallTime::now().toSec() - last_pub_time_;

      // Have we time to interpolate the output?
      double factor = 0.0;
      if (elapsed_time < time_to_interpolate)
      {
        factor = (time_to_interpolate - elapsed_time) / time_to_interpolate;
        tf::Vector3 t_last = last_pub_odom_.getOrigin();
        tf::Vector3 t_curr = corrected_odom.getOrigin();
        double x = factor*t_last.x() + (1-factor)*t_curr.x();
        double y = factor*t_last.y() + (1-factor)*t_curr.y();
        double z = factor*t_last.z() + (1-factor)*t_curr.z();

        // Re-write the output odometry
        tf::Quaternion q = corrected_odom.getRotation();
        tf::Vector3 t(x, y, z);
        tf::Transform tmp(q, t);
        corrected_odom = tmp;
      }

      // Publish
      publish(*odom_msg, corrected_odom);
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
  fromROSMsg(*cloud_msg, *pcl_cloud);
  copyPointCloud(*pcl_cloud, pcl_cloud_);

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
  ros::Time timestamp = l_img_msg->header.stamp;

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

    // Set this transformation for the graph object
    graph_.setCamera2Odom(odom2camera_.inverse());

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

      // Slam initialized!
      first_iter_ = false;
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
    // Exit
    return;
  }

  // Insert this new node into libhaloc
  int id_tmp = lc_.setNode(l_img, r_img);

  // Check if node has been inserted
  if (id_tmp < 0)
  {
    // Exit
    ROS_DEBUG("[Localization:] Impossible to save the node due to its poor quality.");
    return;
  }

  // Decide if the position of this node will be computed by SolvePNP or odometry
  tf::Transform corrected_pose = corrected_odom;
  if (params_.refine_neighbors)
  {
    tf::Transform vertex_disp;
    int last_id = graph_.getLastVertexId();
    bool valid = lc_.getLoopClosure(last_id, last_id+1, vertex_disp);
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
  graph_.findClosestNodes(params_.min_neighbor, 3, neighbors);
  for (uint i=0; i<neighbors.size(); i++)
  {
    tf::Transform edge;
    int lc_id = neighbors[i];
    bool valid_lc = lc_.getLoopClosure(cur_id, lc_id, edge, true);
    if (valid_lc)
    {
      //ROS_INFO_STREAM("[Localization:] Node with id " << cur_id << " closes loop with " << lc_id);
      cout << "\033[1;32m[ INFO]: [Localization:] Node " << cur_id << " closes loop with " << lc_id << ".\033[0m\n";
      graph_.addEdge(lc_id, cur_id, edge);
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
    cout << "\033[1;32m[ INFO]: [Localization:] Node " << cur_id << " closes loop with " << lc_id_num << ".\033[0m\n";
    graph_.addEdge(lc_id_num, cur_id, edge);
    any_loop_closure = true;
  }

  // Update the graph if any loop closing
  if (any_loop_closure) graph_.update();

  // Save graph to file and send (if needed)
  graph_.saveToFile();

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
  removeNaNFromPointCloud(*in_cloud, *cloud, indicies);

  // Voxel grid filter (used as x-y surface extraction. Note that leaf in z is very big)
  pcl::ApproximateVoxelGrid<PointRGB> grid;
  grid.setLeafSize(0.005, 0.005, 0.5);
  grid.setDownsampleAllData(true);
  grid.setInputCloud(cloud);
  grid.filter(*cloud);

  return cloud;
}


/** \brief Save and/or send the accumulated cloud depending on the value of 'save_clouds'
 * \param Cloud id
 */
void slam::SlamBase::processCloud(int cloud_id)
{
  // Proceed?
  if (!params_.save_clouds || pcl_cloud_.points.size() == 0) return;

  // Save cloud
  string id = lexical_cast<string>(cloud_id);
  PointCloudRGB::Ptr cloud;
  cloud = filterCloud(pcl_cloud_.makeShared());
  if (cloud->points.size() == 0) return;
  pcl::io::savePCDFileBinary(params_.clouds_dir + id + ".pcd", *cloud);
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

  // Save last published odometry
  last_pub_odom_ = odom;
  last_pub_time_ = ros::WallTime::now().toSec();

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
  string odom_topic, left_topic, right_topic, left_info_topic, right_info_topic, cloud_topic, correction_topic;
  nh_private_.param("pose_frame_id",              pose_params.pose_frame_id,        string("/map"));
  nh_private_.param("pose_child_frame_id",        pose_params.pose_child_frame_id,  string("/robot"));
  nh_private_.param("odom_topic",                 odom_topic,                       string("/odometry"));
  nh_private_.param("left_topic",                 left_topic,                       string("/left/image_rect_color"));
  nh_private_.param("right_topic",                right_topic,                      string("/right/image_rect_color"));
  nh_private_.param("left_info_topic",            left_info_topic,                  string("/left/camera_info"));
  nh_private_.param("right_info_topic",           right_info_topic,                 string("/right/camera_info"));
  nh_private_.param("cloud_topic",                cloud_topic,                      string("/points2"));
  nh_private_.param("correction_topic",           correction_topic,                 string(""));

  // Motion parameters
  nh_private_.param("refine_neighbors",           params.refine_neighbors,          false);
  nh_private_.param("min_displacement",           params.min_displacement,          0.3);
  nh_private_.param("max_correction",             params.max_correction,            5.0);
  nh_private_.param("correction_interp_time",     params.correction_interp_time,    15.0);

  // Log parameters
  nh_private_.param("save_images",                lc_params.save_images,            false);
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

  // Some other graph parameters
  graph_params.save_dir = lc_params.work_dir;
  graph_params.pose_frame_id = pose_params.pose_frame_id;
  graph_params.pose_child_frame_id = pose_params.pose_child_frame_id;
  graph_params.correction_tp = correction_topic;

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
  // Setup the signal handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = stopHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // Advertise/subscribe class messages
  pose_.advertisePoseMsg(nh_private_);
  graph_.advertiseMsgs(nh_private_);
  graph_.subscribeMsgs(nh_);

  // Generic subscriber
  generic_sub_ = nh_.subscribe<nav_msgs::Odometry>(params_.odom_topic, 1, &SlamBase::genericCallback, this);

  // Enable the slam?
  if (params_.enable)
  {
    // Init
    first_iter_ = true;
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
void slam::SlamBase::finalize(int s)
{
  ROS_INFO("[Localization:] Finalizing...");
  lc_.finalize();
  ROS_INFO("[Localization:] Done!");
}
