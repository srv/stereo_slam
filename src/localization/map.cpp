#include <ros/ros.h>
#include "localization/map.h"

namespace slam
{
  Map::Map()
  {
  }

  void Map::addPoints(Frame frame)
  {
    Cloud world_points = frame.getWorldPoints();
    for (uint i=0; i<world_points.size(); i++)
    {
      //map_points_.insert(world_points[i]);
    }
  }
}