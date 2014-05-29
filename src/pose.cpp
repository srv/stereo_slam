#include "pose.h"

/** \brief Class constructor.
  * @return 
  */
stereo_slam::Pose::Pose(){}

/** \brief Advertices the pose message
  * @return 
  * \param Node handle where pose will be adverticed.
  */
void stereo_slam::Pose::adverticePoseMsg(ros::NodeHandle nh)
{
  // Advertice the pose publication
  pose_pub_ = nh.advertise<nav_msgs::Odometry>("corrected_odom", 1);
}

/** \brief Correct the current odometry with the information of the graph.
  * @return The corrected odometry.
  * \param Current odometry.
  * \param Last graph pose.
  * \param The corresponding original odometry for the last graph pose.
  */
tf::Transform stereo_slam::Pose::correctOdom( tf::Transform current_odom, 
                                              tf::Transform last_graph_pose, 
                                              tf::Transform last_graph_odom)
{
  // Odometry diference
  tf::Transform odom_diff = last_graph_odom.inverse() * current_odom;

  // Compute the corrected pose
  return last_graph_odom * odom_diff;
}

/** \brief Publish some pose.
  * @return
  * \param Pose to be publised.
  */
void stereo_slam::Pose::publish(nav_msgs::Odometry odom_msg, tf::Transform pose)
{
  // Publish pose
  if (pose_pub_.getNumSubscribers() > 0)
  {
    nav_msgs::Odometry pose_msg = odom_msg;
    pose_msg.header.stamp = odom_msg.header.stamp;
    pose_msg.header.frame_id = params_.pose_frame_id;
    pose_msg.child_frame_id = params_.pose_child_frame_id;
    tf::poseTFToMsg(pose, pose_msg.pose.pose);
    pose_pub_.publish(pose_msg);
  }
}

