/**
 * @file
 * @brief The cluster class represents one cv::KeyPoints clustering of the camera frame.
 */

#ifndef CLUSTER_H
#define CLUSTER_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace std;

namespace slam
{

class Cluster
{

public:

  /** \brief Class constructor
   */
  Cluster();

  /** \brief Class constructor
   */
  Cluster(int id, int frame_id, tf::Transform camera_pose, vector<cv::KeyPoint> kp_l, vector<cv::KeyPoint> kp_r, cv::Mat orb_desc, cv::Mat sift_desc, vector<cv::Point3f> points);

  /** \brief Computes and returns the 3D points in world coordinates
   * @return the 3D points in world coordinates
   */
  vector<cv::Point3f> getWorldPoints();

  /** \brief Get the cluster id
   */
  inline int getId() const {return id_;}

  /** \brief Get the frame id
   */
  inline int getFrameId() const {return frame_id_;}

  /** \brief Get left cv::KeyPoints
   */
  inline vector<cv::KeyPoint> getLeftKp() const {return kp_l_;}

  /** \brief Get right cv::KeyPoints
   */
  inline vector<cv::KeyPoint> getRightKp() const {return kp_r_;}

  /** \brief Get orb descriptors
   */
  inline cv::Mat getOrb() const {return orb_desc_;}

  /** \brief Get sift descriptors
   */
  inline cv::Mat getSift() const {return sift_desc_;}

  /** \brief Get 3D camera points
   */
  inline vector<cv::Point3f> getPoints() const {return points_;}

  /** \brief Get camera pose
   */
  inline tf::Transform getCameraPose() const {return camera_pose_;}

private:


  int id_; //!> Cluster id

  int frame_id_; //!> Corresponding frame id

  tf::Transform camera_pose_; //!> Camera world position

  vector<cv::KeyPoint> kp_l_; //!> left cv::KeyPoints.

  vector<cv::KeyPoint> kp_r_; //!> left cv::KeyPoints.

  cv::Mat orb_desc_; //!> ORB descriptors
  cv::Mat sift_desc_; //!> Sift descriptors

  vector<cv::Point3f> points_; //!> Stereo 3D points in camera frame

};

} // namespace

#endif // CLUSTER_H