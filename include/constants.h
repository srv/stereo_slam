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

  static const int MIN_INLIERS_LC = 50;

  static const int MAX_INLIERS_LC = 150;

  static const int LC_NEIGHBORS = 10;

  static const int LC_DISCARD_WINDOW = 25;

} // namespace

#endif // CONSTANTS_H