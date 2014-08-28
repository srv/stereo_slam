/**
 * @file
 * @brief Extended g2o graph node including pointcloud boundaries.
 */

#ifndef _VERTEX_H_
#define _VERTEX_H_

#include <g2o/types/slam3d/vertex_se3.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGB                  PointRGB;

namespace slam
{

class Vertex : public g2o::VertexSE3
{

public:

  // Replicate constructor
  explicit Vertex() : g2o::VertexSE3() { }

  // Access specifiers
  inline void setBoundaries(const PointRGB& min_pt, const PointRGB& max_pt)
  {
    min_pt_ = min_pt;
    max_pt_ = max_pt;
  }
  inline void getBoundaries(PointRGB& min_pt, PointRGB& max_pt)
  {
    min_pt = min_pt_;
    max_pt = max_pt_;
  }


private:
  PointRGB min_pt_;
  PointRGB max_pt_;
};

} // namespace

#endif // VERTEX_H