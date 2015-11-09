/**
 * @file
 * @brief Constants
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <ros/package.h>

using namespace std;

namespace slam
{

  static const string WORKING_DIRECTORY = ros::package::getPath("stereo_slam") + "/";

  static const int LC_MIN_INLIERS = 40;

  static const int LC_MAX_INLIERS = 150;

  static const int LC_NEIGHBORS = 6;

  static const int LC_DISCARD_WINDOW = 15;

  static const float TRACKING_MIN_OVERLAP = 85;

  /*
  DEFAULT VALUES ARE:
  LC_MIN_INLIERS        = 40
  LC_MAX_INLIERS        = 150
  LC_NEIGHBORS          = 6
  LC_DISCARD_WINDOW     = 15
  TRACKING_MIN_OVERLAP  = 85
  */

} // namespace

#endif // CONSTANTS_H