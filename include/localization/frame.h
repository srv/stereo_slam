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

  /** \brief Get left image
   */
  inline Mat getLeftImg() const {return l_img_;}

  /** \brief Get right image
   */
  inline Mat getRightImg() const {return r_img_;}

  /** \brief Get left keypoints
   */
  inline vector<KeyPoint> getLeftKp() const {return l_kp_;}

  /** \brief Get right keypoints
   */
  inline vector<KeyPoint> getRightKp() const {return r_kp_;}

  /** \brief Get left descriptors
   */
  inline Mat getLeftDesc() const {return l_desc_;}

  /** \brief Get 3D
   */
  inline vector<Point3f> get3D() const {return points_3d_;}

protected:

  void extractKp();

  void extractDesc();

private:

  Mat l_img_; //!> Left image
  Mat r_img_; //!> Right image

  vector<KeyPoint> l_kp_; //!> Left keypoints.
  vector<KeyPoint> r_kp_; //!> Right keypoints.

  Mat l_desc_; //!> Left descriptors.
  Mat r_desc_; //!> Right descriptors.

  vector<Point3f> points_3d_;  //!> Stereo 3D points.

};

} // namespace

#endif // TRACKING_H