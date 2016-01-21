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

using namespace std;
using namespace pcl;

typedef PointXYZ                     PointXYZ;
typedef PointXYZRGB                  PointRGB;
typedef PointCloud<PointXYZ>         PointCloudXYZ;
typedef PointCloud<PointRGB>         PointCloudRGB;

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
  Frame(cv::Mat l_img, cv::Mat r_img, image_geometry::StereoCameraModel camera_model, double timestamp);

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

  /** \brief Set frame id
   * \param vector of 3D points
   */
  inline void setId(const int& id){id_ = id;}

  /** \brief Set 3D
   * \param vector of 3D points
   */
  inline void setCameraPoints(const vector<cv::Point3f>& points_3d){camera_points_ = points_3d;}

  /** \brief Set camera pose
   * \param camera pose
   */
  inline void setCameraPose(const tf::Transform& camera_pose){camera_pose_ = camera_pose;}

  /** \brief Set pointcloud
   * \param pointcloud
   */
  inline void setPointCloud(const PointCloudRGB::Ptr pointcloud){pointcloud_ = pointcloud;}

  /** \brief Set number of inliers with the previous frame
   * \param number of inliers between current frame and previous
   */
  inline void setInliersNumWithPreviousFrame(const int& num_inliers){num_inliers_with_prev_frame_ = num_inliers;}

  /** \brief Get frame id
   */
  inline int getId() const {return id_;}

  /** \brief Get camera pose
   */
  inline tf::Transform getCameraPose() const {return camera_pose_;}

  /** \brief Return the clustering for the current frame
   */
  inline vector< vector<int> > getClusters() const {return clusters_;}

  /** \brief Return the clustering for the current frame
   */
  inline vector<Eigen::Vector4f> getClusterCentroids() const {return cluster_centroids_;}

  /** \brief Get frame timestamp
   */
  inline double getTimestamp() const {return stamp_;}

  /** \brief Get frame pointcloud
   */
  inline PointCloudRGB::Ptr getPointCloud() const {return pointcloud_;}

  /** \brief Get frame timestamp
   */
  inline int getInliersNumWithPreviousFrame() const {return num_inliers_with_prev_frame_;}

  /** \brief Compute sift descriptors
   * @return the matrix of sift descriptors
   */
  cv::Mat computeSift();

  /** \brief Cluster the points
   */
  void regionClustering();

protected:

  /** \brief Search keypoints into region
   * @return the list of keypoints into a certain region
   * \param list of keypoints
   * \param query keypoint
   * \param maximum distance to considerate a keypoint into the region
   */
  vector<int> regionQuery(vector<cv::KeyPoint> *keypoints, cv::KeyPoint *keypoint, float eps);

private:

  int id_; //!> Frame id

  cv::Mat l_img_; //!> Left image
  cv::Mat r_img_; //!> Right image

  vector<cv::KeyPoint> l_kp_; //!> Left keypoints.
  vector<cv::KeyPoint> r_kp_; //!> Right keypoints.

  cv::Mat l_desc_; //!> Left descriptors.
  cv::Mat r_desc_; //!> Right descriptors.

  vector<cv::Point3f> camera_points_; //!> Stereo 3D points in camera frame

  vector< vector<int> > clusters_; //!> Keypoints clustering

  vector<Eigen::Vector4f> cluster_centroids_; //!> Central point for every cluster

  tf::Transform camera_pose_; //!> Camera world position for this frame

  double stamp_; //!> Store the frame timestamp

  PointCloudRGB::Ptr pointcloud_; //!> The pointcloud for this frame

  int num_inliers_with_prev_frame_; //!> Number of inliers between this frame and the previous

};

} // namespace

#endif // FRAME_H