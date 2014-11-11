#include "reconstruction/online/viewer.h"


/** \brief Class constructor. Reads node parameters and initialize some properties.
  */
reconstruction::Viewer::Viewer(){}


/** \brief Start the viewer
  */
void reconstruction::Viewer::start()
{
  // Create the viewer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ptr (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_ = viewer_ptr;

  // Configure
  viewer_->addCoordinateSystem(0.1);
  viewer_->initCameraParameters();

  // Make viewer interactive
  while (!viewer_->wasStopped())
  {
    viewer_->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds (100000));
  }
}

/** \brief Stop the viewer
  */
void reconstruction::Viewer::stop()
{
  viewer_->close();
}