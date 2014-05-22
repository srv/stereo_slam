/**
 * @file
 * @brief Iterative closest point refinement from navigation data
 */

#ifndef ICP_REFINEMENT_BASE_H
#define ICP_REFINEMENT_BASE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace stereo_slam
{

class IcpRefinementBase
{

public:

	// Constructor
  IcpRefinementBase(ros::NodeHandle nh, ros::NodeHandle nhp);

  struct Params
  {
    // Topic parameters
    int queue_size;                  //!> Indicate the maximum number of messages encued.

    // default settings
    Params () {
      queue_size                  = 5;
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
  bool initializeIcpRefinement();
  void readParameters();
  void msgsCallback(const sensor_msgs::PointCloud2ConstPtr& points_msg,
                    const geometry_msgs::PoseStampedConstPtr& pose_msg);

private:

  // Topic properties
  message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub_;
  ros::Publisher pose_pub_;

  // Topic sync properties
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, 
                                                          geometry_msgs::PoseStamped> ApproximatePolicy;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ApproximateSync> approximate_sync_;

  /// Stores parameters
  Params params_;
};

} // namespace

#endif // ICP_REFINEMENT_BASE_H