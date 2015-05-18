#define PCL_NO_PRECOMPILE

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


// Custom point type XYZ with the weight property
struct PointRGBID
{
  float x;
  float y;
  float z;
  float rgb;
  int id;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;   // make sure our new allocators are aligned

  // Default values
  PointRGBID () {
    x = 0.0;
    y = 0.0;
    z = 0.0;
    rgb = 0.0;
    id = -1;
  }
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment
POINT_CLOUD_REGISTER_POINT_STRUCT (PointRGBID,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
                                   (int, id, id)
);


typedef pcl::PointXYZRGB              PointRGB;
typedef pcl::PointCloud<PointRGB>     PointCloudRGB;
typedef pcl::PointCloud<PointRGBID>   PointCloudRGBID;

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
  bool lock_graph_, lock_acc_, saving_cloud_;

  // Accumulated cloud
  PointCloudRGBID::Ptr acc_p_;
  PointCloudRGB::Ptr acc_;

public:

  /**
   * Class constructor
   */
  Viewer() : nh_(), nh_private_("~"), acc_p_(new PointCloudRGBID), acc_(new PointCloudRGB)
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
    while(lock_graph_);
      lock_graph_ = true;

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
    lock_graph_ = false;
  }


  /** \brief Thread to reconstruct the pointclouds
   */
  void reconstructionThread()
  {
    // Frequency
    ros::WallDuration d(0.1);

    while(true)
    {
      d.sleep();

      // Check lock
      if (lock_graph_) continue;
      lock_graph_ = true;

      // Copy
      vector< pair<int, tf::Transform> > graph_poses = graph_poses_;

      // Unlock
      lock_graph_ = false;

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
        string lock_graph_file = work_dir_ + id_str + ".lock";
        if(fs::exists(lock_graph_file)) continue;

        // Open
        PointCloudRGB::Ptr cloud(new PointCloudRGB);
        if (pcl::io::loadPCDFile<PointRGB> (cloud_filename, *cloud) == -1)
        {
          ROS_WARN_STREAM("[Viewer:] Couldn't read the file: " << cloud_filename);
          continue;
        }

        // First iteration
        if (acc_p_->points.size() == 0)
        {
          // Transform the cloud
          Eigen::Affine3d pose_eigen;
          transformTFToEigen(pose, pose_eigen);
          pcl::transformPointCloud(*cloud, *cloud, pose_eigen);

          // Add the id to the cloud
          PointCloudRGBID::Ptr cloud_with_id(new PointCloudRGBID);
          for (uint k=0; k<cloud->points.size(); k++)
          {
            PointRGBID p;
            p.x = cloud->points[k].x;
            p.y = cloud->points[k].y;
            p.z = cloud->points[k].z;
            p.rgb = cloud->points[k].rgb;
            p.id = id;
            cloud_with_id->push_back(p);
          }

          // Make this the accumulated
          pcl::copyPointCloud(*cloud_with_id, *acc_p_);
          continue;
        }

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
            ROS_INFO_STREAM("[Viewer:] Updating cloud " << id << ".");

            // Retrieve the points of this cloud
            vector<int> idx_roi;
            pcl::PassThrough<PointRGBID> pass1;
            pass1.setFilterFieldName("id");
            pass1.setFilterLimits(id, id);
            pass1.setInputCloud(acc_p_);
            pass1.filter(idx_roi);
            PointCloudRGBID::Ptr cloud_update(new PointCloudRGBID);
            pcl::copyPointCloud(*acc_p_, idx_roi, *cloud_update);

            // Correct the cloud pose
            tf::Transform correction = v_pose.inverse() * pose;
            Eigen::Affine3d correction_eigen;
            transformTFToEigen(correction.inverse(), correction_eigen);
            pcl::transformPointCloud(*cloud_update, *cloud_update, correction_eigen);

            // Remove the points of this cloud
            pcl::PassThrough<PointRGBID> pass2;
            pass2.setFilterFieldName("id");
            pass2.setFilterLimits(id, id);
            pass2.setFilterLimitsNegative(true);
            pass2.setInputCloud(acc_p_);
            pass2.filter(*acc_p_);

            // Add the updated points
            *acc_p_ += *cloud_update;

            // Save
            viewer_poses_[viewer_idx].second = pose;
          }
        }
        else
        {
          // This cloud is not in the viewer, insert.

          // Transform the cloud
          Eigen::Affine3d pose_eigen;
          transformTFToEigen(pose, pose_eigen);
          pcl::transformPointCloud(*cloud, *cloud, pose_eigen);

          // Reduce the size of the accumulated cloud to speed-up the process
          PointRGB min_pt, max_pt;
          pcl::getMinMax3D(*cloud, min_pt, max_pt);
          vector<int> idx_roi;
          pcl::PassThrough<PointRGBID> pass;
          pass.setFilterFieldName("x");
          pass.setFilterLimits(min_pt.x, max_pt.x);
          pass.setFilterFieldName("y");
          pass.setFilterLimits(min_pt.y, max_pt.y);
          pass.setFilterFieldName("z");
          pass.setFilterLimits(min_pt.z, max_pt.z);
          pass.setInputCloud(acc_p_);
          pass.filter(idx_roi);

          // Extract the interesting part of the accumulated cloud
          PointCloudRGB::Ptr acc_roi(new PointCloudRGB);
          pcl::copyPointCloud(*acc_p_, idx_roi, *acc_roi);

          // Merge current cloud with the accumulated (add only new points)
          PointCloudRGBID::Ptr accepted_points(new PointCloudRGBID);
          pcl::KdTreeFLANN<PointRGB> kdtree;
          kdtree.setInputCloud(acc_roi);
          for (uint n=0; n<cloud->points.size(); n++)
          {
            // Get the cloud point (XY)
            PointRGB sp = cloud->points[n];

            // Search the closest point
            vector<int> idx_vec;
            vector<float> squared_dist;
            if (kdtree.nearestKSearch(sp, 1, idx_vec, squared_dist) > 0)
            {
              float dist = sqrt(squared_dist[0]);
              if (dist > 2*max_dist_)
              {
                PointRGBID p;
                p.x = sp.x;
                p.y = sp.y;
                p.z = sp.z;
                p.rgb = sp.rgb;
                p.id = id;
                accepted_points->push_back(p);
              }
            }
          }
          ROS_INFO_STREAM("[Viewer:] Adding " << accepted_points->points.size() << " to the accumulated cloud...");

          // Accumulate cloud
          *acc_p_ += *accepted_points;

          // Save
          viewer_poses_.push_back(make_pair(id, pose));
        }

        // Store for visualization
        if (lock_acc_) continue;
        lock_acc_ = true;
        pcl::copyPointCloud(*acc_p_, *acc_);
        lock_acc_ = false;
      }
    }
  }


  /** \brief Thread to visualize the pointclouds
   */
  void viewerThread()
  {
    // Create the visualizer
    pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");

    // Add a coordinate system to screen
    viewer.addCoordinateSystem(0.1);
    viewer.registerKeyboardCallback( &Viewer::keyboardEventOccurred, *this );

    // Frequency
    ros::WallDuration d(0.01);

    while(true)
    {
      d.sleep();
      viewer.spinOnce(1);

      if (lock_acc_) continue;
      lock_acc_ = true;

      // Check if accumulated is empty
      if (acc_->points.size() == 0)
      {
        lock_acc_ = false;
        continue;
      }

      // Initialize the camera view
      if(!viewer_initialized_)
      {
        // Set camera position
        Eigen::Vector4f xyz_centroid;
        EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
        pcl::computeMeanAndCovarianceMatrix(*acc_, covariance_matrix, xyz_centroid);
        viewer.initCameraParameters();
        viewer.setCameraPosition(xyz_centroid(0), xyz_centroid(1), xyz_centroid(2)-5.0, 0, -11, 0);

        // First iteration
        pcl::visualization::PointCloudColorHandlerRGBField<PointRGB> color_handler(acc_);
        viewer.addPointCloud(acc_, color_handler, "cloud");
        viewer_initialized_ = true;
      }

      // Update
      viewer.updatePointCloud(acc_, "cloud");

      // Unlock
      lock_acc_ = false;
    }
  }


  /** \brief Keyboard event to save the current cloud
   */
  void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
  {
    if (event.getKeySym() == "space" && event.keyDown()) {

      ROS_INFO("[Viewer:] Saving pointcloud, please wait...");

      // Check if already saving
      if (saving_cloud_) return;
      saving_cloud_ = true;

      // Check lock
      while(lock_acc_);
      lock_acc_ = true;

      // Ok, save the cloud
      string cloud_filename = work_dir_  + "output.pcd";
      if (pcl::io::savePCDFileBinary(cloud_filename, *acc_) == 0)
        ROS_INFO_STREAM("[Viewer:] Pointcloud saved to: " << cloud_filename);
      else
        ROS_ERROR_STREAM("[Viewer:] Problem saving " << cloud_filename);

      lock_acc_ = false;
      saving_cloud_ = false;
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
    // Init
    lock_graph_ = false;
    lock_acc_ = false;
    viewer_initialized_ = false;
    saving_cloud_ = false;

    // Create the message callback
    graph_sub_ = nh_.subscribe<stereo_slam::GraphData>(graph_topic_, 1, &Viewer::msgsCallback, this);

    // Voxel size
    voxel_size_ = 0.005;

    // Maximum distance from point to voxel
    max_dist_ = sqrt( (voxel_size_*voxel_size_)/2 );
  }

};


int main(int argc, char** argv)
{
  // Start the node
  ros::init(argc, argv, "viewer");
  Viewer node;

  // Start the reconstruction thread
  boost::thread reconstruction_thread( boost::bind(&Viewer::reconstructionThread, &node) );

  // Start the viewer thread
  boost::thread viewer_thread( boost::bind(&Viewer::viewerThread, &node) );

  // ROS spin
  ros::MultiThreadedSpinner spinner(0);
  spinner.spin();

  // Join, delete and exit
  reconstruction_thread.join();
  viewer_thread.join();
  return 0;
}

