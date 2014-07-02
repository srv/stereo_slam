/**
 * @file
 * @brief Stereo slam using visual odometry and g2o optimization (presentation).
 */

#ifndef BASE_H
#define BASE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/image_encodings.h>
#include <libhaloc/lc.h>
#include "pose.h"
#include "../common/graph.h"

namespace enc = sensor_msgs::image_encodings;

using namespace std;
using namespace cv;

namespace slam
{

class SlamBase
{

public:

	// Constructor
  SlamBase(ros::NodeHandle nh, ros::NodeHandle nhp);

  // Finalize stereo slam node
  void finalize();

  struct Params
  {
    // Motion parameters
    double min_displacement;         //!> Minimum odometry displacement between poses to be saved as graph vertices.

    // Default settings
    Params () {
      min_displacement            = 0.2;
    }
  };

  /**
   * @param params new parameters
   */
  inline void setParams(const Params& params)
  {
    params_ = params;
  }

  /**
   * @return current parameters
   */
  inline Params params() const { return params_; }

protected:

	// Node handlers
	ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Protected functions and callbacks
  bool init();
  void readParameters();
  void msgsCallback(  const nav_msgs::Odometry::ConstPtr& odom_msg,
                      const sensor_msgs::ImageConstPtr& l_img_msg,
                      const sensor_msgs::ImageConstPtr& r_img_msg,
                      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                      const sensor_msgs::CameraInfoConstPtr& r_info_msg);
  bool getImages( sensor_msgs::Image l_img_msg,
                  sensor_msgs::Image r_img_msg,
                  sensor_msgs::CameraInfo l_info_msg,
                  sensor_msgs::CameraInfo r_info_msg,
                  Mat &l_img, Mat &r_img);

private:

  // Topic properties
  image_transport::SubscriberFilter left_sub_, right_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_, right_info_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;

  // Topic sync properties
  typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry,
                                                    sensor_msgs::Image,
                                                    sensor_msgs::Image,
                                                    sensor_msgs::CameraInfo,
                                                    sensor_msgs::CameraInfo> ExactPolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  boost::shared_ptr<ExactSync> exact_sync_;

  Params params_;                   //!> Stores parameters
  haloc::LoopClosure lc_;           //!> Loop closure object
  slam::Pose pose_;                 //!> Pose object
  slam::Graph graph_;               //!> Graph object
  bool first_iter_;                 //!> Indicates first iteration
};

} // namespace

#endif // BASE_H