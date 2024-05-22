/**
 * @file
 * @brief The frame class represents each stereo camera frame (presentation).
 */

#ifndef FRAME_HPP
#define FRAME_HPP

#include <ros/ros.h>
#include <image_geometry/stereo_camera_model.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <pcl/point_types.h>

typedef pcl::PointXYZ                     PointXYZ;
typedef pcl::PointXYZRGB                  PointRGB;
typedef pcl::PointCloud<PointXYZ>         PointCloudXYZ;
typedef pcl::PointCloud<PointRGB>         PointCloudRGB;

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
  Frame(cv::Mat l_img, cv::Mat r_img, image_geometry::StereoCameraModel camera_model, ros::Time timestamp, std::string feature_detector);

  /** \brief Get left image
   */
  inline cv::Mat getLeftImg() const {return l_img_;}

  /** \brief Get right image
   */
  inline cv::Mat getRightImg() const {return r_img_;}

  /** \brief Get left keypoints
   */
  inline std::vector<cv::KeyPoint> getLeftKp() const {return l_kp_;}

  /** \brief Set left keypoints
   * \param vector of keypoints
   */
  inline void setLeftKp(const std::vector<cv::KeyPoint>& l_kp){l_kp_ = l_kp;}

  /** \brief Get right keypoints
   */
  inline std::vector<cv::KeyPoint> getRightKp() const {return r_kp_;}

  /** \brief Get left non-filtered keypoints
   */
  inline std::vector<cv::KeyPoint> getNonFilteredLeftKp() const {return l_nonfiltered_kp_;}

  /** \brief Get right non-filtered keypoints
   */
  inline std::vector<cv::KeyPoint> getNonFilteredRightKp() const {return r_nonfiltered_kp_;}

  /** \brief Get left descriptors
   */
  inline cv::Mat getLeftDesc() const {return l_desc_;}

  /** \brief Set left descriptors
   * \param vector of descriptors
   */
  inline void setLeftDesc(const cv::Mat& l_desc){l_desc_ = l_desc;}

  /** \brief Get stereo matches
   */
  inline std::vector<cv::DMatch> getMatches() const {return matches_filtered_;}

  /** \brief Get 3D in camera frame
   */
  inline std::vector<cv::Point3f> getCameraPoints() const {return camera_points_;}

  /** \brief Set frame id
   * \param vector of 3D points
   */
  inline void setId(const int& id){id_ = id;}

  /** \brief Set 3D
   * \param vector of 3D points
   */
  inline void setCameraPoints(const std::vector<cv::Point3f>& points_3d){camera_points_ = points_3d;}

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

  /** \brief Set sigma with the previous frame
   * \param sigma matrix between current frame and previous
   */
  inline void setSigmaWithPreviousFrame(const cv::Mat& sigma){sigma_with_prev_frame_ = sigma;}

  /** \brief Get frame id
   */
  inline int getId() const {return id_;}

  /** \brief Get camera pose
   */
  inline tf::Transform getCameraPose() const {return camera_pose_;}

  /** \brief Return the clustering for the current frame
   */
  inline std::vector< std::vector<int> > getClusters() const {return clusters_;}

  /** \brief Return the clustering for the current frame
   */
  inline std::vector<Eigen::Vector4f> getClusterCentroids() const {return cluster_centroids_;}

  /** \brief Get frame timestamp
   */
  inline ros::Time getTimestamp() const {return stamp_;}

  /** \brief Get frame pointcloud
   */
  inline PointCloudRGB::Ptr getPointCloud() const {return pointcloud_;}

  /** \brief Get frame inliers with previous frame
   */
  inline int getInliersNumWithPreviousFrame() const {return num_inliers_with_prev_frame_;}

  /** \brief Get frame sigma with previous frame
   */
  inline cv::Mat getSigmaWithPreviousFrame() const {return sigma_with_prev_frame_;}

  /** \brief Get the time needed to extract the visual features from stereoscopic images
   */
  inline double getFeatureExtractionTimeConsumption() const {return feature_extraction_time_;}

  /** \brief Get the time needed to match features between the frames of a stereo pair
   */
  inline double getStereoMatchingTimeConsumption() const {return stereo_matching_time_;}

  /** \brief Get the time needed to compute the 3D points of the scene
   */
  inline double getCompute3DPointsTimeConsumption() const {return compute_3D_points_time_;}

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
  std::vector<int> regionQuery(std::vector<cv::KeyPoint> *keypoints, cv::KeyPoint *keypoint, float eps);

private:

  int id_; //!> Frame id

  cv::Mat l_img_; //!> Left image
  cv::Mat r_img_; //!> Right image

  std::string feature_detector_;

  std::vector<cv::KeyPoint> l_kp_; //!> Left keypoints.
  std::vector<cv::KeyPoint> r_kp_; //!> Right keypoints.
  
  std::vector<cv::KeyPoint> l_nonfiltered_kp_; //!> Left non-filtered keypoints.
  std::vector<cv::KeyPoint> r_nonfiltered_kp_; //!> Right non-filtered keypoints.

  cv::Mat l_desc_; //!> Left descriptors.
  cv::Mat r_desc_; //!> Right descriptors.

  std::vector<cv::DMatch> matches_filtered_; //!> Filtered stereo matches

  std::vector<cv::Point3f> camera_points_; //!> Stereo 3D points in camera frame

  std::vector< std::vector<int> > clusters_; //!> Keypoints clustering

  std::vector<Eigen::Vector4f> cluster_centroids_; //!> Central point for every cluster

  tf::Transform camera_pose_; //!> Camera world position for this frame

  ros::Time stamp_; //!> Store the frame timestamp

  PointCloudRGB::Ptr pointcloud_; //!> The pointcloud for this frame

  ros::Publisher pub_kp_; //!> Keypoints publisher

  int num_inliers_with_prev_frame_; //!> Number of inliers between this frame and the previous

  cv::Mat sigma_with_prev_frame_; //!> The sigma value with previous frame

  static constexpr float STEREO_EPIPOLAR_THRESH = 1.0; //!> Stereo epipolar threshold

  double feature_extraction_time_ = 0.0; //!> Time needed to extract the visual features from stereoscopic images
  double stereo_matching_time_ = 0.0; //!> Time needed to match features between the frames of a stereo pair
  double compute_3D_points_time_ = 0.0; //!> Time needed to compute the 3D points of the scene

};

} // namespace

#endif // FRAME_HPP