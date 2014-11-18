#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include "reconstruction/online/viewer.h"


/** \brief Class constructor. Reads node parameters and initialize some properties.
  */
reconstruction::Viewer::Viewer(){}


/** \brief Sets the receiver
  * \param the receiver object
  */
void reconstruction::Viewer::setReceiver(reconstruction::Receiver receiver)
{
  receiver_ = receiver;
}


/** \brief Timer event callback for the interaction with slam node
  * \param the timer event
  */
void reconstruction::Viewer::buildCallback(const ros::WallTimerEvent& event)
{
  if (lock_timer_) return;
  lock_timer_ = true;

  ROS_INFO_STREAM("[KKKKKKKKKKKKKKKKKK:] BUILDDDDDDDDDDDDDDDDDDDDDDDDDDDD CALLBACK");

  // Update current pointcloud positions
  updateCloudPoses();

  // Get new pointclouds
  getNewClouds();


  // Update the viewer with the new information
  // TODO


  lock_timer_ = false;
}


/** \brief Update the poses of the clouds list.
  */
void reconstruction::Viewer::updateCloudPoses()
{
  // Loop over the existing pointclouds and update its pose
  for (uint i=0; i<clouds_.size(); i++)
  {
    Cloud cloud = clouds_[i];

    // Get the cloud pose
    tf::Transform new_pose;
    receiver_.getNodePose(cloud.id, new_pose);

    // Check if the new pose has changed significantly compared with the current one
    double pose_dist = tools::Tools::poseDiff(cloud.pose, new_pose);
    ROS_INFO_STREAM("[KKKKKKKKKKKKKKKKKK:] Displacement of node " << cloud.id << ": " << pose_dist);
    if (pose_dist < params_.min_pose_change) continue;

    // The pose has changed significantly.
    cloud.pose = new_pose;
    cloud.has_changed = true;
    clouds_[i] = cloud;
    ROS_INFO_STREAM("[KKKKKKKKKKKKKKKKKK:] New node updated");
  }
}


/** \brief Get new clouds from the receiver
  */
void reconstruction::Viewer::getNewClouds()
{
  bool exist = true;
  while (exist)
  {
    // Get the last cloud id
    string last_id = "-1";
    if (clouds_.size() > 0)
    {
      Cloud c = clouds_[clouds_.size()-1];
      last_id = c.id;
    }

    // The requested cloud id
    int receiver_id_int = boost::lexical_cast<int>(last_id) + 1;
    string receiver_id_str = boost::lexical_cast<string>(receiver_id_int);

    // Get the node (if any)
    tf::Transform new_pose;
    bool exist = receiver_.getNodePose(receiver_id_str, new_pose);

    // There is
    if (exist) {

      // Compute the cloud centroid and radius
      double radius;
      PointXY centroid;
      computeGeometry(receiver_id_str, centroid, radius);

      // Insert the new cloud into the list
      Cloud c(receiver_id_str, new_pose, centroid, radius, true);
      clouds_.push_back(c);
      ROS_INFO_STREAM("[KKKKKKKKKKKKKKKKKK:] New cloud inserted into the viewer: " << receiver_id_str << " | Radius: " << radius);
    }
  }
}


void reconstruction::Viewer::insertCloud()
{
  // TODO
}


void reconstruction::Viewer::updateCloud(string id_str)
{
  // Id to int
  int id_int = boost::lexical_cast<int>(id_str);

  // Compute the overlap between this cloud and all previous
  vector<int> overlapping_clouds;
  for (int i=id_int-1; i>=0; i--)
  {
    // TODO
  }

  // Build the accumulated cloud of all the overlapping clouds
  for (uint i=0; i<overlapping_clouds.size(); i++)
  {
    // TODO
  }

  // Compute the non-overlapping region of the current pointcloud
  // TODO

  // Update this pointcloud into the viewer

}


/** \brief Compute cloud geometries: centroid and radius
  * \param cloud id
  * \param cloud centroid
  * \param cloud radius (bounding circle in x-y)
  */
void reconstruction::Viewer::computeGeometry(string id, PointXY &centroid, double &radius)
{
  // Open the cloud (if exists)
  string cloud_path = params_.work_dir + id + ".pcd";
  PointCloudRGB::Ptr cloud(new PointCloudRGB);
  if (pcl::io::loadPCDFile<PointRGB> (cloud_path, *cloud) == -1)
  {
    ROS_WARN_STREAM("[Reconstruction:] Couldn't read the cloud: " << cloud_path);
    return;
  }

  // Get the centroid
  Eigen::Vector4f cent;
  pcl::compute3DCentroid(*cloud, cent);
  centroid.x = cent[0];
  centroid.y = cent[1];

  // Get the radius
  Eigen::Vector4f max_pt;
  pcl::getMaxDistance(*cloud, cent, max_pt);
  radius = sqrt( pow(centroid.x-max_pt[0], 2) + pow(centroid.y-max_pt[1], 2) );
}


/** \brief Update visualization thread
  */
void reconstruction::Viewer::updateVisualization()
{
  while (!viewer_->wasStopped())
  {
    viewer_->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds (100000));
  }
}


/** \brief Start the viewer
  */
void reconstruction::Viewer::start()
{
  // Init
  clouds_.clear();

  // Create the viewer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ptr (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_ = viewer_ptr;

  // Configure
  viewer_->addCoordinateSystem(0.1);
  viewer_->initCameraParameters();

  // Used to lock the timer
  lock_timer_ = false;

  // Initialize the timer for updating the viewer
  timer_update_ = params_.nh_private.createWallTimer(ros::WallDuration(0.5),
                                                     &reconstruction::Viewer::buildCallback,
                                                     this);

  // Make viewer interactive
  visualization_thread_ = boost::thread(&reconstruction::Viewer::updateVisualization, this);
}


/** \brief Stop the viewer
  */
void reconstruction::Viewer::stop()
{
  visualization_thread_.join();
  timer_update_.stop();
  viewer_->close();
}