#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include "reconstruction/online/viewer.h"


/** \brief Class constructor. Reads node parameters and initialize some properties.
  */
reconstruction::Viewer::Viewer(){}


/** \brief Update the viewers
  */
void reconstruction::Viewer::update()
{
  ROS_INFO_STREAM("[Reconstruction:] KKKKKKKKKKKKKKKKKKKKKK");
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

  // Make viewer interactive
  visualization_thread_ = boost::thread(&reconstruction::Viewer::updateVisualization, this);
}


/** \brief Stop the viewer
  */
void reconstruction::Viewer::stop()
{
  visualization_thread_.join();
  viewer_->close();
}