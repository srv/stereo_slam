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

  static const int MIN_INLIERS = 40;

  static const int MAX_INLIERS = 80;

  static const int LC_NEIGHBORS = 15;

  static const int LC_GROUP_RANGE = 5;

} // namespace

#endif // CONSTANTS_H