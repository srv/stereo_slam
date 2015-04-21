#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>
#include <boost/thread.hpp>
#include "common/tools.h"
#include "stereo_slam/Correction.h"

using namespace std;
using namespace tools;
using namespace boost;

typedef pcl::PointXYZRGB              PointRGB;
typedef pcl::PointCloud<PointRGB>     PointCloudRGB;

/** \brief Catches the Ctrl+C signal.
  */
void stopHandler(int s)
{
  printf("Caught signal %d\n", s);
  exit(1);
}

class Viewer
{

private:

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Topic subscribers
  ros::Subscriber pose_sub_;

  // Node parameters
  string work_dir_;
  string correction_topic_;
  float voxel_size_;
  float max_dist_;
  bool viewer_initialized_;

  // Accumulated cloud
  PointCloudRGB::Ptr acc_;

public:

  /**
   * Class constructor
   */
  Viewer() : nh_(), nh_private_("~"), acc_(new PointCloudRGB)
  {
    // Setup the signal handler
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = stopHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    // Read parameters
    readParameters();

    // Init class
    init();
  }


  /** \brief Messages callback. This function is called when synchronized vertex and cloud
  * message are received.
  * @return
  * \param correction_msg message of type stereo_slam::VertexCorrection
  */
  // void msgsCallback(const stereo_slam::VertexCorrection::ConstPtr& correction_msg)
  // {
  //   // Check
  //   int node_id = correction_msg->node_id;

  //   // Extract the pose
  //   tf::Vector3 t(correction_msg->x, correction_msg->y, correction_msg->z);
  //   tf::Quaternion q(correction_msg->qx, correction_msg->qy, correction_msg->qz, correction_msg->qw);
  //   tf::Transform pose(q, t);

  //   // Load the cloud
  //   string cloud_filename = work_dir_ + lexical_cast<string>(node_id) + ".pcd";
  //   PointCloudRGB::Ptr cloud(new PointCloudRGB);
  //   if (pcl::io::loadPCDFile<PointRGB> (cloud_filename, *cloud) == -1)
  //   {
  //     ROS_WARN_STREAM("[Viewer:] Couldn't read the file: " << cloud_filename);
  //     return;
  //   }

  //   // Transform the cloud
  //   Eigen::Affine3d pose_eigen;
  //   transformTFToEigen(pose, pose_eigen);
  //   pcl::transformPointCloud(*cloud, *cloud, pose_eigen);

  //   if (acc_->points.size() == 0)
  //   {
  //     pcl::copyPointCloud(*cloud, *acc_);
  //     return;
  //   }

  //   // Reduce the size of the accumulated cloud to speed-up the process
  //   PointRGB min_pt, max_pt;
  //   pcl::getMinMax3D(*cloud, min_pt, max_pt);
  //   vector<int> idx_roi;
  //   pcl::PassThrough<PointRGB> pass;
  //   pass.setFilterFieldName("x");
  //   pass.setFilterLimits(min_pt.x, max_pt.x);
  //   pass.setFilterFieldName("y");
  //   pass.setFilterLimits(min_pt.y, max_pt.y);
  //   pass.setFilterFieldName("z");
  //   pass.setFilterLimits(min_pt.z, max_pt.z);
  //   pass.setInputCloud(acc_);
  //   pass.filter(idx_roi);

  //   // Extract the interesting part of the accumulated cloud
  //   PointCloudRGB::Ptr acc_roi(new PointCloudRGB);
  //   pcl::copyPointCloud(*acc_, idx_roi, *acc_roi);

  //   // Merge current cloud with the accumulated (add only new points)
  //   ROS_INFO("[Viewer:] Merging cloud with accumulated...");
  //   PointCloudRGB::Ptr accepted_points(new PointCloudRGB);
  //   pcl::KdTreeFLANN<PointRGB> kdtree;
  //   kdtree.setInputCloud(acc_roi);
  //   for (uint n=0; n<cloud->points.size(); n++)
  //   {
  //     // Get the cloud point (XY)
  //     PointRGB sp = cloud->points[n];

  //     // Search the closest point
  //     vector<int> idx_vec;
  //     vector<float> squared_dist;
  //     if (kdtree.nearestKSearch(sp, 1, idx_vec, squared_dist) > 0)
  //     {
  //       float dist = sqrt(squared_dist[0]);
  //       if (dist > 2*max_dist_)
  //         accepted_points->push_back(sp);
  //     }
  //   }
  //   ROS_INFO_STREAM("[Viewer:] Adding " << accepted_points->points.size() << " to the accumulated cloud...");

  //   // Accumulate cloud
  //   *acc_ += *accepted_points;
  // }


  /** \brief Thread to align the pointclouds
   */
  void viewerThread()
  {
    // Create the visualizer
    pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");

    // Add a coordinate system to screen
    viewer.addCoordinateSystem(0.1);

    // Frequency
    ros::WallDuration d(0.01);

    while(true)
    {
      d.sleep();
      viewer.spinOnce(1);

      // If no cloud received yet, continue
      if(acc_->points.size() == 0)
        continue;

      // Delete the previous point cloud
      viewer.removePointCloud("cloud");

      // Initialize the camera view
      if(!viewer_initialized_)
      {
        Eigen::Vector4f xyz_centroid;
        EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
        pcl::computeMeanAndCovarianceMatrix(*acc_, covariance_matrix, xyz_centroid);
        viewer.initCameraParameters();
        viewer.setCameraPosition(xyz_centroid(0), xyz_centroid(1), xyz_centroid(2)+3.0, 0, -1, 0);
        ROS_INFO_STREAM("[Viewer:] Point cloud viewer camera initialized in: [" <<
          xyz_centroid(0) << ", " << xyz_centroid(1) << ", " << xyz_centroid(2)+3.0 << "]");
        viewer_initialized_ = true;
      }
      // Show the point cloud
      pcl::visualization::PointCloudColorHandlerRGBField<PointRGB> color_handler(acc_);
      viewer.addPointCloud(acc_, color_handler, "cloud");
    }
  }


  /** \brief Reads the node parameters
   */
  void readParameters()
  {
    nh_private_.param("work_dir",           work_dir_,          string(""));
    nh_private_.param("correction_topic",   correction_topic_,  string(""));

    // Init workspace
    if (work_dir_[work_dir_.length()-1] != '/')
      work_dir_ += "/";
  }


  /** \brief Initialize class
   */
  void init()
  {
    // Create the callback
    //pose_sub_ = nh_.subscribe<stereo_slam::VertexCorrection>(correction_topic_, 1, &Viewer::msgsCallback, this);

    // Voxel size
    voxel_size_ = 0.005;

    // Maximum distance from point to voxel
    max_dist_ = sqrt( (voxel_size_*voxel_size_)/2 );

    // Set uninitialized
    viewer_initialized_ = false;
  }

};


int main(int argc, char** argv)
{
  // Start the node
  ros::init(argc, argv, "viewer");
  Viewer node;

  // Start the align thread
  boost::thread viewer_thread( boost::bind(&Viewer::viewerThread, &node) );

  // ROS spin
  ros::MultiThreadedSpinner spinner(0);
  spinner.spin();

  // Join, delete and exit
  viewer_thread.join();
  return 0;
}

