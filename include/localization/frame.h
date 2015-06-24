/**
 * @file
 * @brief The frame class represents each stereo camera frame (presentation).
 */

#ifndef FRAME_H
#define FRAME_H

#include <ros/ros.h>
#include <image_geometry/stereo_camera_model.h>
#include <tf/transform_datatypes.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

using namespace std;
using namespace cv;

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
  Frame(Mat l_img, Mat r_img, image_geometry::StereoCameraModel camera_model);

  /** \brief Set id
   * \param the frame id
   */
  inline void setId(const int& id){id_ = id;}

  /** \brief Get the frame id
   */
  inline int getId() const {return id_;}

  /** \brief Get left image
   */
  inline Mat getLeftImg() const {return l_img_;}

  /** \brief Get right image
   */
  inline Mat getRightImg() const {return r_img_;}

  /** \brief Get left keypoints
   */
  inline vector<KeyPoint> getLeftKp() const {return l_kp_;}

  /** \brief Set left keypoints
   * \param vector of keypoints
   */
  inline void setLeftKp(const vector<KeyPoint>& l_kp){l_kp_ = l_kp;}

  /** \brief Get right keypoints
   */
  inline vector<KeyPoint> getRightKp() const {return r_kp_;}

  /** \brief Get left descriptors
   */
  inline Mat getLeftDesc() const {return l_desc_;}

  /** \brief Set left descriptors
   * \param vector of descriptors
   */
  inline void setLeftDesc(const Mat& l_desc){l_desc_ = l_desc;}

  /** \brief Get 3D
   */
  inline vector<Point3f> get3D() const {return points_3d_;}

  /** \brief Set 3D
   * \param vector of 3D points
   */
  inline void set3D(const vector<Point3f>& points_3d){points_3d_ = points_3d;}

  /** \brief Set the inliers value
   * \param number of inliers of this frame to some specific fixed frame
   */
  inline void setInliers(const int& inliers){inliers_to_fixed_frame_ = inliers;}

  /** \brief Get the inliers value
   */
  inline int getInliers() const {return inliers_to_fixed_frame_;}

  /** \brief Set estimated camera pose
   * \param camera pose
   */
  inline void setEstimatedPose(const tf::Transform& estimated_pose){estimated_pose_ = estimated_pose;}

  /** \brief Get estimated camera pose
   */
  inline tf::Transform getEstimatedPose() const {return estimated_pose_;}

  /** \brief Set the frame graph neighbors
   * \param list of graph neighbors
   */
  inline void setGraphNeighbors(const vector<int>& graph_neighbors){graph_neighbors_ = graph_neighbors;}

  /** \brief Get frame graph neighbors
   */
  inline vector<int> getGraphNeighbors() const {return graph_neighbors_;}

  /** \brief Set odometry camera pose
   * \param camera pose
   */
  inline void setOdometryPose(const tf::Transform& odom_pose){odom_pose_ = odom_pose;}

  /** \brief Get odometry camera pose
   */
  inline tf::Transform getOdometryPose() const {return odom_pose_;}

  /** \brief Compute sift descriptors
   * @return the matrix of sift descriptors
   */
  Mat computeSift();

private:

  int id_; //!> Frame id

  Mat l_img_; //!> Left image
  Mat r_img_; //!> Right image

  vector<KeyPoint> l_kp_; //!> Left keypoints.
  vector<KeyPoint> r_kp_; //!> Right keypoints.

  Mat l_desc_; //!> Left descriptors.
  Mat r_desc_; //!> Right descriptors.

  vector<Point3f> points_3d_; //!> Stereo 3D points.

  int inliers_to_fixed_frame_; //!> Number of inliers of this frame to some specific fixed frame

  tf::Transform estimated_pose_; //!> Camera position for this frame computed by solvePNPransac

  tf::Transform odom_pose_; //!> Odometry position for this frame (could not coincide with the camera)

  vector<int> graph_neighbors_; //!> Closest graph neighbors

};

} // namespace

#endif // TRACKING_H