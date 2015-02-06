#include "reconstruction/base.h"
#include <boost/filesystem.hpp>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>

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

  // Number of nearest neighbors
  int K = 1;

  // Camera parameters. FIXME: extract from camera info
  float baseline = 0.12;
  float focal = 775.312;
  float dis_dev = 0.1;
  float const_unc = dis_dev / (baseline * focal);

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

    // Get the maximum and minimum of the accumulated and current cloud
    PointCloudXYZ::Ptr acc_xyz(new PointCloudXYZ);
    pcl::copyPointCloud(*acc, *acc_xyz);
    PointXYZ min_acc, max_acc;
    pcl::getMinMax3D(*acc_xyz, min_acc, max_acc);

    // Convert the accumulated cloud to PointXY. So, we are supposing
    // the robot is navigating parallel to the surface.
    PointCloudXY::Ptr acc_xy(new PointCloudXY);
    pcl::copyPointCloud(*acc, *acc_xy);

    // Search the closest neighbor in the projected accumulated cloud.
    pcl::KdTreeFLANN<PointXY> kdtree;
    kdtree.setInputCloud(acc_xy);

    // Search the closest neighbor over the line of sight of the current point in the accumulated cloud.
    pcl::KdTreeFLANN<PointXYZRGBW> kdtree2;
    kdtree2.setInputCloud(acc);

    // Merge the current cloud with the accumulated
    for (uint n=0; n<cloud->points.size(); n++)
    {
      // Get the cloud point (XY)
      PointXY sp;
      sp.x = cloud->points[n].x;
      sp.y = cloud->points[n].y;

      // Check if this point is inside or in the border of the current accumulated cloud
      vector<int> point_idx_NKN_search(K);
      vector<float> point_NKN_squared_distance(K);
      if (kdtree.radiusSearch(sp, 2*max_dist, point_idx_NKN_search, point_NKN_squared_distance, K) > 0)
      {
        // Get the corresponding point into the accumulated
        PointXYZRGBW p = acc->points[ point_idx_NKN_search[0] ];

        // Is this the border?
        if (point_NKN_squared_distance[0] > max_dist*max_dist)
        {
          // This is in the border, filter the z part and add it

          // Get the uncertainty for this z value
          float delta_z = cloud->points[n].z * cloud->points[n].z * const_unc;

          // Build the point
          p.x = cloud->points[n].x;
          p.y = cloud->points[n].y;
          p.z = (cloud->points[n].z + p.z) / 2;
          p.rgb = cloud->points[n].rgb;
          p.w = delta_z;

          // Add the point
          acc->push_back(p);
        }
        else
        {
          // This point is inside the accumulated cloud, apply the weighted function and update

          // Get the uncertainty for this z value
          float delta_z = p.z * p.z * const_unc;

          // TODO: apply to x, y and rgb?
          p.z = (p.w*p.z + delta_z*cloud->points[n].z) / (p.w + delta_z);

          // Update the weight
          p.w = p.w + delta_z;

          // Update the point
          acc->points[ point_idx_NKN_search[0] ] = p;
        }
      }
      else
      {
        // This points is not inside the voxels of the accumulated cloud. So, may be
        // the point is a new point, far away from the accumulated borders, or may be
        // the point is inside a hole of the accumulated cloud.

        // Get the uncertainty for this z value
        float delta_z = cloud->points[n].z * cloud->points[n].z * const_unc;

        // Extract the point
        PointXYZRGBW p;
        p.x = cloud->points[n].x;
        p.y = cloud->points[n].y;
        p.z = cloud->points[n].z;
        p.rgb = cloud->points[n].rgb;
        p.w = delta_z;

        if ((sp.x < max_acc.x && sp.x > min_acc.x) ||
            (sp.y < max_acc.y && sp.y > min_acc.y))
        {
          // Point inside
          // TODO
        }

        // Add the point
        acc->push_back(p);
      }






      /*
      if (sp.x > max_acc.x || sp.x < min_acc.x ||
          sp.y > max_acc.y || sp.y < min_acc.y)
      {
        // The point is outside the accumulated cloud

        // Extract the point
        PointXYZRGBW p;
        p.x = cloud->points[n].x;
        p.y = cloud->points[n].y;
        p.z = cloud->points[n].z;
        p.rgb = cloud->points[n].rgb;

        // Check if it is close to the border
        vector<int> point_idx_NKN_search2(K);
        vector<float> point_NKN_squared_distance2(K);
        if (kdtree2.radiusSearch(p, max_dist, point_idx_NKN_search2, point_NKN_squared_distance2, K) > 0)
        {
          // The point is close to the border, so merge it!
          PointXYZRGBW acc_p = acc->points[ point_idx_NKN_search2[0] ];
          p.x = (p.x + acc_p.x) / 2;
          p.y = (p.y + acc_p.y) / 2;
          p.z = (p.z + acc_p.z) / 2;
        }

        // Add the point
        acc->push_back(p);
      }
      else
      {
        // The points is inside the accumulated cloud

        // Search this point into the voxels of the accumulated cloud
        vector<int> point_idx_NKN_search(K);
        vector<float> point_NKN_squared_distance(K);
        if (kdtree.radiusSearch(sp, max_dist, point_idx_NKN_search, point_NKN_squared_distance, K) > 0)
        {
          // Get the corresponding point into the accumulated
          PointXYZRGBW acc_p = acc->points[ point_idx_NKN_search[0] ];

          // Get the uncertainty for this z value
          float delta_z = acc_p.z * acc_p.z * const_unc;

          // Apply the weighted function
          // TODO: apply to x, y and rgb?
          acc_p.z = (acc_p.w*acc_p.z + delta_z*cloud->points[n].z) / (acc_p.w + delta_z);

          // Update the weight
          acc_p.w = acc_p.w + delta_z;

          // Update the point
          acc->points[ point_idx_NKN_search[0] ] = acc_p;
        }
        else
        {
          // The point is not outside nor in the voxels of the accumulated cloud, thus this point is inside a hole!

          // Extract the point
          PointXYZRGBW p;
          p.x = cloud->points[n].x;
          p.y = cloud->points[n].y;
          p.z = cloud->points[n].z;
          p.rgb = cloud->points[n].rgb;

          // Follow the line of sight to find other points intersecting.
          // The distance to the camera point of view (0,0,0) is:
          float dist = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);

          // The number of partitions
          int partitions = round(dist / voxel_size);
          float x_spacing = p.x/partitions;
          float y_spacing = p.y/partitions;
          float z_spacing = p.z/partitions;

          // Loop the line of sight
          vector<int> dist_idx;
          vector<float> dist_vec;
          vector<PointXYZ> points_vec;
          for (uint k=1; k<partitions; k++)
          {
            PointXYZRGBW p_los;
            p_los.z = p.z - z_spacing*k;
            if (p_los.z < min_acc.z)
              break;

            if (p.x < 0)
              p_los.x = p.x + x_spacing*k;
            else
              p_los.x = p.x - x_spacing*k;
            if (p.y < 0)
              p_los.y = p.y + x_spacing*k;
            else
              p_los.y = p.y - x_spacing*k;

            vector<int> point_idx_NKN_search2(K);
            vector<float> point_NKN_squared_distance2(K);
            if (kdtree2.nearestKSearch(p_los, K, point_idx_NKN_search2, point_NKN_squared_distance2) > 0)
            {
              dist_idx.push_back(point_idx_NKN_search2[0]);
              dist_vec.push_back(point_NKN_squared_distance2[0]);
              PointXYZ p_tmp;
              p_tmp.x = p_los.x;
              p_tmp.y = p_los.y;
              p_tmp.z = p_los.z;
              points_vec.push_back(p_tmp);
            }
          }

          // Correct the point if needed
          if (dist_vec.size() > 0)
          {
            // Get the minimum distance of the line of sight to the closest accumulated point.
            vector<float>::iterator result = min_element(dist_vec.begin(), dist_vec.end());
            int min_idx = distance(dist_vec.begin(), result);
            float min_dist_to_acc = dist_vec[min_idx];

            // This is a point which is back to the surface and must be corrected
            p.x = points_vec[min_idx].x;
            p.y = points_vec[min_idx].y;
            p.z = points_vec[min_idx].z;
            acc->push_back(p);
          }

          acc->push_back(p);
        }
      }
      */

    }

    // Return the acc to its original pose
    pcl::transformPointCloud(*acc, *acc, tfn0_eigen.inverse());
  }

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