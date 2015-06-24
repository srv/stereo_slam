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

  static const string WORKING_DIRECTORY = ros::package::getPath("stereo_slam") + "/execution/";

  static const int MIN_INLIERS = 30;

  static const int MAX_INLIERS = 50;

} // namespace

#endif // CONSTANTS_H