#include "localization/pose.h"

/** \brief Class constructor.
  * @return
  */
slam::Pose::Pose(){}

/** \brief Advertises the pose message
  * @return
  * \param Node handle where pose will be advertised.
  */
void slam::Pose::advertisePoseMsg(ros::NodeHandle nh)
{
  // Advertise the pose publication
  pose_pub_ = nh.advertise<nav_msgs::Odometry>("odometry", 1);
}

/** \brief Correct the pose with the information of the graph.
  * @return The corrected pose.
  * \param Current pose.
  * \param Last graph pose.
  * \param The corresponding original pose for the last graph pose.
  */
tf::Transform slam::Pose::correctPose(tf::Transform pose,
                                      tf::Transform last_graph_pose,
                                      tf::Transform last_graph_odom)
{
  // Odometry difference
  tf::Transform odom_diff = last_graph_odom.inverse() * pose;

  // Compute the corrected pose
  return last_graph_pose * odom_diff;
}

/** \brief Publish pose.
  * @return
  * \param original odometry message.
  * \param Corrected odometry to be published.
  * \param true to publish the graph pose.
  */
void slam::Pose::publish(nav_msgs::Odometry odom_msg, tf::Transform pose)
{
  // Broadcast the transformation
  frame_to_child_.sendTransform(tf::StampedTransform(pose,
                                                     odom_msg.header.stamp,
                                                     params_.pose_frame_id,
                                                     params_.pose_child_frame_id));

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

