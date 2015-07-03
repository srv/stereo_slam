#include <ros/ros.h>

#include "frame.h"
#include "constants.h"
#include "tools.h"
#include "ldb.h"

using namespace tools;

namespace slam
{

  Frame::Frame() {}

  Frame::Frame(cv::Mat l_img, cv::Mat r_img, image_geometry::StereoCameraModel camera_model)
  {
    l_img.copyTo(l_img_);
    r_img.copyTo(r_img_);

    // Convert images to grayscale
    cv::Mat l_img_gray, r_img_gray;
    cv::cvtColor(l_img, l_img_gray, CV_RGB2GRAY);
    cv::cvtColor(r_img, r_img_gray, CV_RGB2GRAY);

    // Detect keypoints
    vector<cv::KeyPoint> l_kp, r_kp;
    cv::ORB orb(1000, 1.2f, 8, 14, 0, 2, 0, 14);
    orb(l_img_gray, noArray(), l_kp, noArray(), false);
    orb(r_img_gray, noArray(), r_kp, noArray(), false);

    // Extract descriptors
    cv::Mat l_desc, r_desc;
    LDB extractor_;
    extractor_.compute(l_img_gray, l_kp, l_desc, 0);
    extractor_.compute(r_img_gray, r_kp, r_desc, 0);

    // Left/right matching
    vector<cv::DMatch> matches, matches_filtered;
    Tools::ratioMatching(l_desc, r_desc, 0.8, matches);

    // Filter matches by epipolar
    for (size_t i=0; i<matches.size(); ++i)
    {
      if (abs(l_kp[matches[i].queryIdx].pt.y - r_kp[matches[i].trainIdx].pt.y) < 1.0)
        matches_filtered.push_back(matches[i]);
    }

    // Compute 3D points
    l_kp_.clear();
    r_kp_.clear();
    camera_points_.clear();
    l_desc_.release();
    r_desc_.release();
    for (size_t i=0; i<matches_filtered.size(); ++i)
    {
      cv::Point3d world_point;
      int l_idx = matches_filtered[i].queryIdx;
      int r_idx = matches_filtered[i].trainIdx;

      cv::Point2d l_point = l_kp[l_idx].pt;
      cv::Point2d r_point = r_kp[r_idx].pt;

      double disparity = l_point.x - r_point.x;
      camera_model.projectDisparityTo3d(l_point, disparity, world_point);

      // Save
      if ( isfinite(world_point.x) && isfinite(world_point.y) && isfinite(world_point.z) && world_point.z > 0)
      {
        l_kp_.push_back(l_kp[l_idx]);
        r_kp_.push_back(r_kp[r_idx]);
        l_desc_.push_back(l_desc.row(l_idx));
        r_desc_.push_back(r_desc.row(r_idx));
        camera_points_.push_back(world_point);
      }
    }
  }

  cv::Mat Frame::computeSift()
  {
    cv::Mat sift;
    if (l_img_.cols == 0)
      return sift;

    cv::initModule_nonfree();
    cv::Ptr<cv::DescriptorExtractor> cv_extractor;
    cv_extractor = cv::DescriptorExtractor::create("SIFT");
    cv_extractor->compute(l_img_, l_kp_, sift);

    return sift;
  }

  void Frame::regionClustering()
  {
    // Normal estimation
    NormalEstimationOMP<PointXYZ, Normal> ne;
    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>());
    ne.setInputCloud(world_points);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.1);
    ne.compute(*normals);

    // Clustering
    clusters_.clear();
    RegionGrowing<PointXYZ, Normal> reg;
    reg.setMinClusterSize(20);
    reg.setMaxClusterSize(world_points->points.size());
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(20);
    reg.setInputCloud(world_points);
    reg.setInputNormals(normals);
    reg.extract(clusters_);

    // Extract the cluster center
    cluster_centroids_.clear();
    for (uint i=0; i<clusters_.size(); i++)
    {
      Cloud::Ptr region(new Cloud);
      copyPointCloud(*world_points, clusters_[i], *region);
      Eigen::Vector4f centroid;
      compute3DCentroid(*region, centroid);
      cluster_centroids_.push_back(centroid);
    }
  }

} //namespace slam