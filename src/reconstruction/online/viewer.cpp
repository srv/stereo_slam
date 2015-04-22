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
#include <boost/filesystem.hpp>
#include "common/tools.h"
#include "stereo_slam/GraphData.h"

using namespace std;
using namespace tools;
using namespace boost;

namespace fs=filesystem;

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
  ros::Subscriber graph_sub_;

  // Node parameters
  string work_dir_;
  string graph_topic_;
  float voxel_size_;
  float max_dist_;
  bool viewer_initialized_;
  vector< pair<int, tf::Transform> > graph_poses_;
  vector< pair<int, tf::Transform> > viewer_poses_;
  bool lock_;

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


  /** \brief Called when a graph message is received.
   * @return
   * \param graph_msg message of type stereo_slam::GraphData
   */
  void msgsCallback(const stereo_slam::GraphData::ConstPtr& graph_msg)
  {
    // Lock
    while(lock_) {}
      lock_ = true;

    // Reset the data
    graph_poses_.clear();

    // Parse message
    vector<int> id = graph_msg->id;
    for(uint i=0; i<id.size(); i++)
    {
      tf::Vector3 t(graph_msg->x[i], graph_msg->y[i], graph_msg->z[i]);
      tf::Quaternion q(graph_msg->qx[i], graph_msg->qy[i], graph_msg->qz[i], graph_msg->qw[i]);
      tf::Transform pose(q, t);
      graph_poses_.push_back(make_pair(id[i], pose));
    }

    // Unlock
    lock_ = false;
  }


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

      // Check lock
      if (lock_) continue;
      lock_ = true;

      // Copy
      vector< pair<int, tf::Transform> > graph_poses = graph_poses_;

      // Unlock
      lock_ = false;

      // Initialize the camera view
      if(!viewer_initialized_)
      {
        Eigen::Vector4f xyz_centroid;
        EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
        pcl::computeMeanAndCovarianceMatrix(*acc_, covariance_matrix, xyz_centroid);
        viewer.initCameraParameters();
        viewer.setCameraPosition(xyz_centroid(0), xyz_centroid(1), xyz_centroid(2)-5.0, -1, -1, 0);
        ROS_INFO_STREAM("[Viewer:] Point cloud viewer camera initialized in: [" <<
          xyz_centroid(0) << ", " << xyz_centroid(1) << ", " << xyz_centroid(2)+3.0 << "]");
        viewer_initialized_ = true;
      }

      // If no data received yet, continue
      if (graph_poses.size() == 0) continue;

      // Loop the graph
      for (uint i=0; i<graph_poses.size(); i++)
      {
        // The data
        int id = graph_poses[i].first;
        tf::Transform pose = graph_poses[i].second;

        // Check if this cloud is on the workspace
        string id_str = lexical_cast<string>(id);
        string cloud_filename = work_dir_ + id_str + ".pcd";
        if (!fs::exists(cloud_filename)) continue;

        // Wait until unlock
        string lock_file = work_dir_ + id_str + ".lock";
        if(fs::exists(lock_file)) continue;

        // Open
        PointCloudRGB::Ptr cloud(new PointCloudRGB);
        if (pcl::io::loadPCDFile<PointRGB> (cloud_filename, *cloud) == -1)
        {
          ROS_WARN_STREAM("[Viewer:] Couldn't read the file: " << cloud_filename);
          continue;
        }

        // Transform the cloud
        Eigen::Affine3d pose_eigen;
        transformTFToEigen(pose, pose_eigen);
        pcl::transformPointCloud(*cloud, *cloud, pose_eigen);

        // Search this cloud in the viewer
        bool is_in_viewer = false;
        int viewer_idx = -1;
        for (uint j=0; j<viewer_poses_.size(); j++)
        {
          if (id == viewer_poses_[j].first)
          {
            is_in_viewer = true;
            viewer_idx = j;
            break;
          }
        }

        if (is_in_viewer)
        {
          // This cloud is in the viewer. Move it if needed.
          tf::Transform v_pose = viewer_poses_[viewer_idx].second;
          double pose_diff = Tools::poseDiff(pose, v_pose);
          if (pose_diff > 0.01)
          {
            // Update
            ROS_INFO_STREAM("[Viewer:] Updating visualization of cloud " << id_str << ".");
            viewer.updatePointCloud(cloud, id_str);
            viewer_poses_[viewer_idx].second = pose;
          }
        }
        else
        {
          // This cloud is not in the viewer, insert.
          ROS_INFO_STREAM("[Viewer:] Adding cloud " << id_str << ".");
          pcl::visualization::PointCloudColorHandlerRGBField<PointRGB> color_handler(cloud);
          viewer.addPointCloud(cloud, color_handler, id_str);
          viewer_poses_.push_back(make_pair(id, pose));
        }
      }
    }
  }


  /** \brief Reads the node parameters
   */
  void readParameters()
  {
    nh_private_.param("work_dir",      work_dir_,     string(""));
    nh_private_.param("graph_topic",   graph_topic_,  string(""));

    // Init workspace
    if (work_dir_[work_dir_.length()-1] != '/')
      work_dir_ += "/";
  }


  /** \brief Initialize class
   */
  void init()
  {
    // Unlock
    lock_ = false;

    // Create the callback
    graph_sub_ = nh_.subscribe<stereo_slam::GraphData>(graph_topic_, 1, &Viewer::msgsCallback, this);

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

