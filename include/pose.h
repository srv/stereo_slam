/**
 * @file
 * @brief Pose class.
 */

#ifndef POSE_H
#define POSE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace std;

namespace stereo_slam
{

class Pose
{

public:

	// Constructor
  Pose();

  struct Params
  {
    string pose_frame_id;             //!> Pose frame id for publisher
    string pose_child_frame_id;       //!> Base frame id for publisher

    // default settings
    Params () {
      pose_frame_id               = "/map";
      pose_child_frame_id         = "/robot";
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

  // Advertices the pose message
  void adverticePoseMsg(ros::NodeHandle nh);

  // Correct odometry
  tf::Transform correctOdom(tf::Transform current_odom, tf::Transform last_graph_pose, tf::Transform last_graph_odom);

  // Publish pose
  void publish(nav_msgs::Odometry odom_msg, tf::Transform pose);

private:

  // Stores parameters
  Params params_;

  // Pose publisher
  ros::Publisher pose_pub_;

};

} // namespace

#endif // POSE_H