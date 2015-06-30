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
using namespace cv;

namespace slam
{

class Tracking;

class Publisher
{

public:

  /** \brief Class constructor
   */
  Publisher();

  /** \brief Update the image to be published
   * \param The tracker object contains all the information needed to draw the image
   */
  void update(Tracking *tracker);

  /** \brief Draw and publish stereo matchings
   * \param The frame containing the stereo information
   */
  void drawStereoMatchings(const Frame frame);

  /** \brief Update the image to be published
   * \param Fixed frame
   * \param Current frame
   * \param Matches between fixed and current frame
   * \param Inliers between fixed and current frame
   */
  void drawTrackerMatchings(const Frame fixed_frame,
                            const Frame current_frame,
                            const vector<DMatch> matches,
                            const vector<int> inliers);

  /** \brief Draw and publish the keypoint clustering
   * \param The frame containing the clustering information
   */
  void drawKeypointsClustering(const Frame frame);

private:

  ros::Publisher pub_stereo_matching_; //!> Publisher for the left/right matching.

  ros::Publisher pub_tracker_matching_; //!> Publisher for the tracker matching (fixed to current frame).

  ros::Publisher pub_clustering_; //!> Publisher for the tracker matching (fixed to current frame).

};

} // namespace

#endif // PUBLISHER_H