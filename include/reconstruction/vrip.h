/**
 * @file
 * @brief Custom implementation of the VRIP algorithm of Curless & Levoy
 */

#ifndef VRIP_H
#define VRIP_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <pcl_ros/point_cloud.h>

using namespace std;

typedef pcl::PointXYZ             PointXYZ;
typedef pcl::PointXYZRGB          PointRGB;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;

namespace reconstruction
{

  class Vrip
  {
    public:

      /** \brief Class constructor
      */
      Vrip();

      /** \brief Merges pointclouds
      */
      void merge();

    protected:

      /** \brief 
       */
      tf::Transform readCloudPose(int cloud_id);
  };

}

#endif // VRIP_H

