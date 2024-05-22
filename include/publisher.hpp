/**
 * @file
 * @brief The frame publisher class is responsive to publish the image visualizations for debugging purposes (presentation).
 */

#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <boost/lexical_cast.hpp>

#include "tracking.hpp"
#include "frame.hpp"

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
   * \param The frame object contains all the information needed to draw the image
   */
  void publishClustering(const Frame frame);

  /** \brief Publish the stereo matching debug image
   * \param The frame object contains all the information needed to draw the image
   */
  void publishStereoMatches(const Frame frame);

protected:

  /** \brief Draw and publish the keypoint clustering
   * \param The frame containing the clustering information
   */
  void drawKeypointsClustering(const Frame frame);

  /** \brief Draw and publish the stereo matches
   * \param The frame containing the clustering information
   */
  void drawStereoMatches(const Frame frame);

  /** \brief Publishes images
   * \param The ROS publisher, the cv image and the ROS timestamp
   */
  void publishImage(ros::Publisher pub, cv::Mat image, ros::Time stamp);

private:

  ros::Publisher pub_clustering_; //!> Publisher for the frame clustering

  ros::Publisher pub_stereo_matches_img_; //!> Publisher for the frame stereo matches

  ros::Publisher pub_stereo_matches_num_; //!> Publisher for the frame stereo matches

};

} // namespace

#endif // PUBLISHER_HPP