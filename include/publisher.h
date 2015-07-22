/**
 * @file
 * @brief The frame publisher class is responsive to publish the image visualizations for debugging purposes (presentation).
 */

#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include "tracking.h"
#include "frame.h"

using namespace std;

namespace slam
{

class Tracking;

class Publisher
{

public:

  /** \brief Class constructor
   */
  Publisher();

  /** \brief Publish the clustering debug images
   * \param The tracker object contains all the information needed to draw the image
   */
  void publishClustering(const Frame frame);

protected:

  /** \brief Draw and publish the keypoint clustering
   * \param The frame containing the clustering information
   */
  void drawKeypointsClustering(const Frame frame);

private:

  ros::Publisher pub_clustering_; //!> Publisher for the tracker matching (fixed to current frame).

};

} // namespace

#endif // PUBLISHER_H