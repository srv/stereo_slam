#include "reconstruction/base.h"
#include <boost/filesystem.hpp>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>

namespace fs=boost::filesystem;


/** \brief Catches the Ctrl+C signal.
  */
void stopHandler(int s)
{
  printf("Caught signal %d\n",s);
  exit(1);
}


/** \brief Class constructor. Reads node parameters and initialize some properties.
  * @return
  * \param nh public node handler
  * \param nhp private node handler
  */
reconstruction::ReconstructionBase::ReconstructionBase(
  ros::NodeHandle nh, ros::NodeHandle nhp) : nh_(nh), nh_private_(nhp)
{
  // Setup the signal handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = stopHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}


/** \brief Translate the clouds to the corresponding pose of the graph
  */
void reconstruction::ReconstructionBase::translateClouds()
{
  // Read the graph poses
  vector< pair<string, tf::Transform> > cloud_poses;
  if (!readPoses(cloud_poses)) return;

  // Init the kdtree
  const float radius = 0.005;
  pcl::KdTreeFLANN<PointXY> kdtree;

  for (uint i=0; i<cloud_poses.size(); i++)
  {
    string file_idx = cloud_poses[i].first;
    ROS_INFO_STREAM("[Reconstruction:] Processing cloud " << file_idx.substr(0,file_idx.length()-4) << "/" << cloud_poses.size()-1);

    // Read the current pointcloud.
    string cloud_filename = params_.clouds_dir + cloud_poses[i].first;
    PointCloudRGB::Ptr in_cloud(new PointCloudRGB);
    if (pcl::io::loadPCDFile<PointRGB> (cloud_filename, *in_cloud) == -1)
    {
      ROS_WARN_STREAM("[Reconstruction:] Couldn't read the file: " << cloud_poses[i].first);
      continue;
    }

    ROS_INFO("Filtering");

    // Remove nans
    vector<int> indicies;
    PointCloudRGB::Ptr cloud(new PointCloudRGB);
    pcl::removeNaNFromPointCloud(*in_cloud, *cloud, indicies);

    // Voxel grid filter
    pcl::VoxelGrid<PointRGB> grid;
    grid.setLeafSize(0.01, 0.01, 0.01);
    grid.setDownsampleAllData(true);
    grid.setInputCloud(cloud);
    grid.filter(*cloud);

    // Remove isolated points
    pcl::RadiusOutlierRemoval<PointRGB> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.02);
    outrem.setMinNeighborsInRadius(10);
    outrem.filter(*cloud);
    pcl::StatisticalOutlierRemoval<PointRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(20);
    sor.setStddevMulThresh(3.0);
    sor.filter(*cloud);

    pcl::io::savePCDFileBinary(params_.output_dir + "filtered.pcd", *cloud);

    // Normal estimation*
    pcl::NormalEstimation<PointRGB, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointRGB>::Ptr tree(new pcl::search::KdTree<PointRGB>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(50);
    n.compute(*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    // Create search tree*
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // Greedy Projection
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
    pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh());
    ROS_INFO("GreedyProjectionTriangulation");
    gp3.setSearchRadius(0.2);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(50);
    gp3.setMaximumSurfaceAngle(M_PI/2); // 90 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(*triangles);
    pcl::io::saveVTKFile(params_.output_dir + "triangles.vtk", *triangles);

    // Grit projection
    ROS_INFO("GridProjection");
    pcl::PolygonMesh::Ptr gridprojection(new pcl::PolygonMesh());
    pcl::GridProjection<pcl::PointXYZRGBNormal> gp;
    gp.setInputCloud(cloud_with_normals);
    gp.setSearchMethod(tree2);
    gp.setResolution(0.01);
    gp.setPaddingSize(3);
    gp.reconstruct(*gridprojection);

    // Mesh smoothing
    ROS_INFO("MeshSmoothingLaplacianVTK");
    pcl::PolygonMesh output;
    pcl::MeshSmoothingLaplacianVTK vtk;
    vtk.setInputMesh(gridprojection);
    vtk.setNumIter(20000);
    vtk.setConvergence(0.001);
    vtk.setRelaxationFactor(0.001);
    vtk.setFeatureEdgeSmoothing(true);
    vtk.setFeatureAngle(M_PI/5);
    vtk.setBoundarySmoothing(true);
    vtk.process(output);
    pcl::io::saveVTKFile(params_.output_dir + "grid.vtk", output);


    // Transform pointcloud
    //pcl_ros::transformPointCloud(*cloud, *cloud, cloud_poses[i].second);
  }
}

/** \brief Make a greedy triangulation
  */
pcl::PolygonMesh::Ptr reconstruction::ReconstructionBase::greedyProjection(PointCloudRGB::Ptr cloud)
{
  // Normal estimation*
  pcl::NormalEstimation<PointRGB, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<PointRGB>::Ptr tree(new pcl::search::KdTree<PointRGB>);
  tree->setInputCloud(cloud);
  n.setInputCloud(cloud);
  n.setSearchMethod(tree);
  n.setKSearch(50);
  n.compute(*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

  // Create search tree*
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  tree2->setInputCloud(cloud_with_normals);

  // Greedy Projection
  pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh());
  pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
  gp3.setSearchRadius(0.2);
  gp3.setMu(2.5);
  gp3.setMaximumNearestNeighbors(50);
  gp3.setMaximumSurfaceAngle(M_PI/2); // 90 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);
  gp3.setInputCloud(cloud_with_normals);
  gp3.setSearchMethod(tree2);
  gp3.reconstruct(*triangles);
  return triangles;
}

/** \brief Build the 3D
  */
void reconstruction::ReconstructionBase::build3D()
{
  // Read the graph poses
  vector< pair<string, tf::Transform> > cloud_poses;
  if (!readPoses(cloud_poses)) return;

  // Voxel size
  float voxel_size = 0.005;

  // Maximum distance from point to voxel (plus 5% to improve the borders)
  float max_dist = sqrt( (voxel_size*voxel_size)/2 );

  // Camera parameters. FIXME: extract from camera info
  float baseline = 0.12;
  float focal = 775.312;
  float dis_dev = 0.1;
  float const_unc = dis_dev / (baseline * focal);

  // Total of points processed
  int total_points = 0;

  // Load, convert and accumulate every pointcloud
  PointCloudXYZW::Ptr acc(new PointCloudXYZW);
  for (uint i=0; i<cloud_poses.size(); i++)
  {
    string file_idx = cloud_poses[i].first;
    ROS_INFO_STREAM("[Reconstruction:] Processing cloud " << file_idx.substr(0,file_idx.length()-4) << "/" << cloud_poses.size()-1);

    // Read the current pointcloud.
    string cloud_filename = params_.clouds_dir + cloud_poses[i].first;
    PointCloudRGB::Ptr in_cloud(new PointCloudRGB);
    if (pcl::io::loadPCDFile<PointRGB> (cloud_filename, *in_cloud) == -1)
    {
      ROS_WARN_STREAM("[Reconstruction:] Couldn't read the file: " << cloud_poses[i].first);
      continue;
    }

    // Increase the total of points processed
    total_points += in_cloud->points.size();

    ROS_INFO("Filtering");

    // Remove nans
    vector<int> indices;
    PointCloudRGB::Ptr cloud(new PointCloudRGB);
    pcl::removeNaNFromPointCloud(*in_cloud, *cloud, indices);
    indices.clear();

    // Voxel grid filter (used as x-y surface extraction. Note that leaf in z is very big)
    pcl::ApproximateVoxelGrid<PointRGB> grid;
    grid.setLeafSize(voxel_size, voxel_size, 0.5);
    grid.setDownsampleAllData(true);
    grid.setInputCloud(cloud);
    grid.filter(*cloud);

    // Remove isolated points
    pcl::RadiusOutlierRemoval<PointRGB> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.04);
    outrem.setMinNeighborsInRadius(50);
    outrem.filter(*cloud);
    pcl::StatisticalOutlierRemoval<PointRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(40);
    sor.setStddevMulThresh(2.0);
    sor.filter(*cloud);

    ROS_INFO("Merging");

    // First iteration
    if (acc->points.size() == 0)
    {
      // Make this the accumulated
      pcl::copyPointCloud(*cloud, *acc);

      // Init the weights
      for (uint n=0; n<acc->points.size(); n++)
      {
        float delta_z = acc->points[n].z * acc->points[n].z * const_unc;
        acc->points[n].w = delta_z;
      }

      continue;
    }

    // Transform the accumulated cloud to the new cloud frame
    tf::Transform tf0 = cloud_poses[0].second;
    tf::Transform tfn0 = cloud_poses[i].second.inverse()*tf0;
    Eigen::Affine3d tfn0_eigen;
    transformTFToEigen(tfn0, tfn0_eigen);
    pcl::transformPointCloud(*acc, *acc, tfn0_eigen);

    // Convert the accumulated cloud to PointXY. So, we are supposing
    // the robot is navigating parallel to the surface and the camera line of sight is
    // perpendicular to the surface.
    PointCloudXY::Ptr acc_xy(new PointCloudXY);
    pcl::copyPointCloud(*acc, *acc_xy);

    // To search the closest neighbor in the projected accumulated cloud.
    pcl::KdTreeFLANN<PointXY> kdtree;
    kdtree.setInputCloud(acc_xy);

    // Merge the current cloud with the accumulated
    for (uint n=0; n<cloud->points.size(); n++)
    {
      // Get the cloud point (XY)
      PointXY sp;
      sp.x = cloud->points[n].x;
      sp.y = cloud->points[n].y;

      // Get the uncertainty for this z value
      float delta_z = cloud->points[n].z * cloud->points[n].z * const_unc;

      // Extract the point
      PointXYZRGBW p;
      p.x = cloud->points[n].x;
      p.y = cloud->points[n].y;
      p.z = cloud->points[n].z;
      p.rgb = cloud->points[n].rgb;
      p.w = delta_z;

      // Check if this point is inside or in the border of the current accumulated cloud
      int K = 10;
      vector<int> point_idx_NKN_search(K);
      vector<float> point_NKN_squared_distance(K);
      int num_neighbors = kdtree.radiusSearch(sp, 2*max_dist, point_idx_NKN_search, point_NKN_squared_distance, K);
      if (num_neighbors > 0)
      {
        // Filter the z and rgb parts of the point accordingly
        for (int h=0; h<num_neighbors; h++)
        {
          // Get the corresponding point into the accumulated
          PointXYZRGBW p_acc = acc->points[ point_idx_NKN_search[h] ];

          // Filter Z part
          //p.z = (p.w*p.z + p_acc.w*p_acc.z) / (p.w + p_acc.w);
          p.z = (p.z + p_acc.z) / 2;

          // Filter RGB part
          int acc_rgb = *reinterpret_cast<const int*>(&(p_acc.rgb));
          uint8_t acc_r = (acc_rgb >> 16) & 0x0000ff;
          uint8_t acc_g = (acc_rgb >> 8) & 0x0000ff;
          uint8_t acc_b = (acc_rgb) & 0x0000ff;
          int cloud_rgb = *reinterpret_cast<const int*>(&(p.rgb));
          uint8_t cloud_r = (cloud_rgb >> 16) & 0x0000ff;
          uint8_t cloud_g = (cloud_rgb >> 8) & 0x0000ff;
          uint8_t cloud_b = (cloud_rgb) & 0x0000ff;

          cloud_r = (cloud_r + acc_r) / 2;
          cloud_g = (cloud_g + acc_g) / 2;
          cloud_b = (cloud_b + acc_b) / 2;
          //cloud_b = (p.w*cloud_b + p_acc.w*acc_b) / (p.w + p_acc.w);

          int32_t new_rgb = (cloud_r << 16) | (cloud_g << 8) | cloud_b;
          p.rgb = *reinterpret_cast<float*>(&new_rgb);

          // Update the weight
          //p.w = p.w + p_acc.w;
        }

        // Determine if it is a point on the border or not.
        bool is_border = true;
        for (int h=0; h<num_neighbors; h++)
        {
          if (point_NKN_squared_distance[h] < max_dist*max_dist)
          {
            is_border = false;
            break;
          }
        }

        // Is this the border?
        if (is_border)
        {
          // This is in the border, filter the z part and add it

          // Build the new point
          PointXYZRGBW p_new;
          p_new.x = cloud->points[n].x;
          p_new.y = cloud->points[n].y;
          p_new.rgb = cloud->points[n].rgb;
          p_new.z = p.z;
          p_new.w = p.w;

          // Add the point
          acc->push_back(p_new);
        }
        else
        {
          // This point is inside the accumulated cloud, apply the weighted function and update

          // Search the closest point into the accumulated cloud.
          int min_index = min_element(point_NKN_squared_distance.begin(), point_NKN_squared_distance.end()) - point_NKN_squared_distance.begin();

          // Update the point
          PointXYZRGBW p_acc = acc->points[ point_idx_NKN_search[min_index] ];
          p_acc.z = p.z;
          p_acc.w = p.w;
          acc->points[ point_idx_NKN_search[min_index] ] = p_acc;
        }
      }
      else
      {
        // This points is not inside the voxels of the accumulated cloud. So, may be
        // the point is a new point, far away from the accumulated borders, or may be
        // the point is inside a hole of the accumulated cloud.

        // Add the point
        acc->push_back(p);
      }
    }

    // Return the acc to its original pose
    pcl::transformPointCloud(*acc, *acc, tfn0_eigen.inverse());
  }

  ROS_INFO("Filtering output cloud");

  // Remove the weight field
  PointCloudRGB::Ptr acc_rgb(new PointCloudRGB);
  pcl::copyPointCloud(*acc, *acc_rgb);

  // Filter the accumulated cloud
  pcl::ApproximateVoxelGrid<PointRGB> grid;
  grid.setLeafSize(voxel_size, voxel_size, voxel_size);
  grid.setDownsampleAllData(true);
  grid.setInputCloud(acc_rgb);
  grid.filter(*acc_rgb);
  pcl::RadiusOutlierRemoval<PointRGB> outrem_acc;
  outrem_acc.setInputCloud(acc_rgb);
  outrem_acc.setRadiusSearch(0.04);
  outrem_acc.setMinNeighborsInRadius(50);
  outrem_acc.filter(*acc_rgb);
  pcl::StatisticalOutlierRemoval<PointRGB> sor_acc;
  sor_acc.setInputCloud(acc_rgb);
  sor_acc.setMeanK(40);
  sor_acc.setStddevMulThresh(2.0);
  sor_acc.filter(*acc_rgb);

  // TODO: COLOR FILTERING

  // Generate a greedy projection
  /*
  ROS_INFO("[Reconstruction:] Generating Greedy projection...");
  pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh());
  triangles = greedyProjection(acc_rgb);
  */

  // Save accumulated cloud
  ROS_INFO("[Reconstruction:] Saving pointclouds...");
  pcl::io::savePCDFile(params_.work_dir + "reconstruction.pcd", *acc_rgb);
  //pcl::io::saveVTKFile(params_.work_dir + "reconstruction.vtk", *triangles);
  ROS_INFO("[Reconstruction:] Accumulated clouds saved.");
  ROS_INFO_STREAM("[Reconstruction:] Points processed: " << total_points);
}

/** \brief Reads the reconstruction node parameters
  */
void reconstruction::ReconstructionBase::setParameters(string work_dir)
{
  Params params;

  // Operational directories
  if (work_dir[work_dir.length()-1] != '/')
    work_dir += "/";
  params.work_dir = work_dir;
  params.clouds_dir = work_dir + "clouds/";
  params.output_dir = work_dir + "clouds/output/";
  params.graph_file = work_dir + "graph_vertices.txt";
  setParams(params);

  // Create the output directory
  if (fs::is_directory(params.output_dir))
    fs::remove_all(params.output_dir);
  fs::path dir(params.output_dir);
  if (!fs::create_directory(dir))
    ROS_ERROR("[Localization:] ERROR -> Impossible to create the output directory.");
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