#include "reconstruction/offline/base.h"
#include <boost/filesystem.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/transforms.h>

namespace fs=boost::filesystem;

/** \brief Class constructor. Reads node parameters and initialize some properties.
  * @return
  * \param nh public node handler
  * \param nhp private node handler
  */
reconstruction::ReconstructionBase::ReconstructionBase(
  ros::NodeHandle nh, ros::NodeHandle nhp) : nh_(nh), nh_private_(nhp)
{
  // Read the node parameters
  readParameters();
}

/** \brief Build the 3D
  */
void reconstruction::ReconstructionBase::build3D()
{
  // Read the graph poses
  vector< pair<string, tf::Transform> > cloud_poses;
  if (!readPoses(cloud_poses)) return;

  // Init the kdtree
  const float radius = 0.005;
  pcl::KdTreeFLANN<PointXY> kdtree;

  // Init the voxel grid filter
  pcl::VoxelGrid<PointRGB> grid;
  PointCloud::Ptr cloud_downsampled_ptr(new PointCloud);
  grid.setLeafSize(0.003, 0.003, 0.003);
  grid.setDownsampleAllData(true);

  // Load, convert and accumulate every pointcloud
  PointCloud::Ptr acc(new PointCloud);
  for (uint i=0; i<cloud_poses.size(); i++)
  {
    string file_idx = cloud_poses[i].first;
    ROS_INFO_STREAM("[Reconstruction:] Processing cloud " << file_idx.substr(0,file_idx.length()-4) << "/" << cloud_poses.size()-1);

    // Read the current pointcloud.
    string cloud_filename = params_.clouds_dir + cloud_poses[i].first;
    PointCloud::Ptr cloud(new PointCloud);
    if (pcl::io::loadPCDFile<PointRGB> (cloud_filename, *cloud) == -1)
    {
      ROS_WARN_STREAM("[Reconstruction:] Couldn't read the file: " << cloud_poses[i].first);
      continue;
    }

    // Remove isolated points
    pcl::StatisticalOutlierRemoval<PointRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud);

    // Transform pointcloud
    pcl_ros::transformPointCloud(*cloud, *cloud, cloud_poses[i].second);

    // First iteration
    if (acc->points.size() == 0)
    {
      copyPointCloud(*cloud, *acc);
      continue;
    }

    // Convert the accumulated cloud to PointXY
    pcl::PointCloud<PointXY>::Ptr acc_xy(new pcl::PointCloud<PointXY>);
    copyPointCloud(*acc, *acc_xy);

    vector<int> point_idx;
    vector<float> point_distance;
    kdtree.setInputCloud(acc_xy);

    int total = 0;
    int added = 0;
    for (uint i=0; i<cloud->points.size(); i++)
    {
      PointXY cur_point;
      cur_point.x = cloud->points[i].x;
      cur_point.y = cloud->points[i].y;
      if (!isfinite(cur_point.x) || !isfinite(cur_point.y))
        continue;

      total++;
      if (kdtree.radiusSearch(cur_point, radius, point_idx, point_distance) <= 0)
      {
        added++;
        acc->push_back(cloud->points[i]);
      }
    }

    ROS_INFO_STREAM("[Reconstruction:] Original size: " << total << ". Finally added: " << added);

    // Filter the accumulated cloud
    grid.setInputCloud(acc);
    grid.filter(*cloud_downsampled_ptr);
    acc = cloud_downsampled_ptr;
  }

  // Save accumulated cloud
  ROS_INFO("[Reconstruction:] Saving pointcloud...");
  pcl::io::savePCDFile(params_.work_dir + "reconstruction.pcd", *acc);
  ROS_INFO("[Reconstruction:] Accumulated cloud saved.");
}

/** \brief Reads the reconstruction node parameters
  */
void reconstruction::ReconstructionBase::readParameters()
{
  Params params;

  // Operational directories
  string work_dir;
  nh_private_.param("work_dir", work_dir, string(""));
  if (work_dir[work_dir.length()-1] != '/')
    work_dir += "/";
  params.work_dir = work_dir;
  params.clouds_dir = work_dir + "clouds/";
  params.graph_file = work_dir + "graph_vertices.txt";

  setParams(params);
}

/** \brief Reads the poses file and return the vector with the cloud names and the transformations
  * @return true if poses read correctly, false otherwise
  * \param the vector with the cloud filenames and the transformations
  */
bool reconstruction::ReconstructionBase::readPoses(vector< pair<string, tf::Transform> > &cloud_poses)
{
  // Init
  cloud_poses.clear();

  // Wait until poses file is unblocked
  while(fs::exists(params_.work_dir + ".graph.block"));

  // Get the pointcloud poses file
  ifstream poses_file(params_.graph_file.c_str());
  string line;
  while (getline(poses_file, line))
  {
    int i = 0;
    string cloud_name, value;
    double x, y, z, qx, qy, qz, qw;
    istringstream ss(line);
    while(getline(ss, value, ','))
    {
      if (i == 1)
        cloud_name = value + ".pcd";
      else if (i == 5)
        x = boost::lexical_cast<double>(value);
      else if (i == 6)
        y = boost::lexical_cast<double>(value);
      else if (i == 7)
        z = boost::lexical_cast<double>(value);
      else if (i == 8)
        qx = boost::lexical_cast<double>(value);
      else if (i == 9)
        qy = boost::lexical_cast<double>(value);
      else if (i == 10)
        qz = boost::lexical_cast<double>(value);
      else if (i == 11)
        qw = boost::lexical_cast<double>(value);
      i++;
    }
    // Build the pair and save
    tf::Vector3 t(x, y, z);
    tf::Quaternion q(qx, qy, qz, qw);
    tf::Transform transf(q, t);
    cloud_poses.push_back(make_pair(cloud_name, transf));
  }
  return true;
}