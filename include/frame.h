/**
 * @file
 * @brief The frame class represents each stereo camera frame (presentation).
 */

#ifndef FRAME_H
#define FRAME_H

#include <ros/ros.h>
#include <image_geometry/stereo_camera_model.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <pcl/point_types.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d_omp.h>

using namespace std;
using namespace pcl;

typedef PointCloud<PointXYZ> Cloud;

namespace slam
{

class Frame
{

public:

  /** \brief Empty class constructor
   */
  Frame();

  /** \brief Class constructor
   */
  Frame(cv::Mat l_img, cv::Mat r_img, image_geometry::StereoCameraModel camera_model);

  /** \brief Get left image
   */
  inline cv::Mat getLeftImg() const {return l_img_;}

  /** \brief Get right image
   */
  inline cv::Mat getRightImg() const {return r_img_;}

  /** \brief Get left keypoints
   */
  inline vector<cv::KeyPoint> getLeftKp() const {return l_kp_;}

  /** \brief Set left keypoints
   * \param vector of keypoints
   */
  inline void setLeftKp(const vector<cv::KeyPoint>& l_kp){l_kp_ = l_kp;}

  /** \brief Get right keypoints
   */
  inline vector<cv::KeyPoint> getRightKp() const {return r_kp_;}

  /** \brief Get left descriptors
   */
  inline cv::Mat getLeftDesc() const {return l_desc_;}

  /** \brief Set left descriptors
   * \param vector of descriptors
   */
  inline void setLeftDesc(const cv::Mat& l_desc){l_desc_ = l_desc;}

  /** \brief Get 3D in camera frame
   */
  inline vector<cv::Point3f> getCameraPoints() const {return camera_points_;}

  /** \brief Set 3D
   * \param vector of 3D points
   */
  inline void setCameraPoints(const vector<cv::Point3f>& points_3d){camera_points_ = points_3d;}

  /** \brief Set camera pose
   * \param camera pose
   */
  inline void setPose(const tf::Transform& pose){pose_ = pose;}

  /** \brief Get camera pose
   */
  inline tf::Transform getPose() const {return pose_;}

  /** \brief Return the clustering for the current frame
   */
  inline vector<PointIndices> getClusters() const {return clusters_;}

  /** \brief Return the clustering for the current frame
   */
  inline vector<Eigen::Vector4f> getClusterCentroids() const {return cluster_centroids_;}

  /** \brief Compute sift descriptors
   * @return the matrix of sift descriptors
   */
  cv::Mat computeSift();

  /** \brief Cluster the points
   */
  void regionClustering();

private:

  cv::Mat l_img_; //!> Left image
  cv::Mat r_img_; //!> Right image

  vector<cv::KeyPoint> l_kp_; //!> Left keypoints.
  vector<cv::KeyPoint> r_kp_; //!> Right keypoints.

  cv::Mat l_desc_; //!> Left descriptors (orb).
  cv::Mat r_desc_; //!> Right descriptors (orb).

  vector<cv::Point3f> camera_points_; //!> Stereo 3D points in camera frame

  vector<PointIndices> clusters_; //!> 3D points clustering

  vector<Eigen::Vector4f> cluster_centroids_; //!> Central point for every cluster

  tf::Transform pose_; //!> Camera world position for this frame

};

} // namespace

#endif // FRAME_H