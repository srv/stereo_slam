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

  static const int LC_MIN_INLIERS = 20;

  static const int LC_MAX_INLIERS = 80;

  static const int LC_NEIGHBORS = 10;

  static const int LC_DISCARD_WINDOW = 25;

} // namespace

#endif // CONSTANTS_H