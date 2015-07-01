/**
 * @file
 * @brief The cluster class represents one cv::KeyPoints clustering of the camera frame.
 */

#ifndef CLUSTER_H
#define CLUSTER_H

#include <ros/ros.h>

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
  Cluster(int id, vector<cv::KeyPoint> kp, cv::Mat orb_desc, cv::Mat sift_desc, vector<cv::Point3f> points);

  /** \brief Get the frame id
   */
  inline int getId() const {return id_;}

  /** \brief Get cv::KeyPoints
   */
  inline vector<cv::KeyPoint> getKp() const {return kp_;}

  /** \brief Get orb descriptors
   */
  inline cv::Mat getOrb() const {return orb_desc_;}

  /** \brief Get sift descriptors
   */
  inline cv::Mat getSift() const {return sift_desc_;}

  /** \brief Get 3D camera points
   */
  inline vector<cv::Point3f> getPoints() const {return points_;}


private:


  int id_; //!> Cluster id

  vector<cv::KeyPoint> kp_; //!> cv::KeyPoints.

  cv::Mat orb_desc_; //!> Orb descriptors
  cv::Mat sift_desc_; //!> Sift descriptors

  vector<cv::Point3f> points_; //!> Stereo 3D points in camera frame

};

} // namespace

#endif // CLUSTER_H