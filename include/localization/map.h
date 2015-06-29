/**
 * @file
 * @brief The map class represents the slam graph (presentation).
 */

#ifndef MAP_H
#define MAP_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <set>

#include <opencv2/opencv.hpp>

#include "frame.h"

using namespace std;
using namespace cv;

namespace slam
{

class Map
{

public:

  /** \brief Empty class constructor
   */
  Map();

  /** \brief Add points to the map
   * \param the points to be added
   */
  void addPoints(Frame frame);

protected:

  std::set<Point3f*> map_points_; //!> Map points

};

} // namespace

#endif // MAP_H