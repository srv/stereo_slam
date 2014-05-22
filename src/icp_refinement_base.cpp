#include "icp_refinement_base.h"
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include <libpq-fe.h>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "postgresql_interface.h"
#include "utils.h"

/** \brief Class constructor. Reads node parameters and initialize some properties.
  * @return 
  * \param nh public node handler
  * \param nhp private node handler
  */
stereo_slam::IcpRefinementBase::IcpRefinementBase(
  ros::NodeHandle nh, ros::NodeHandle nhp) : nh_(nh), nh_private_(nhp)
{
  // Read the node parameters
  readParameters();

  // Initialize the stereo slam
  initializeIcpRefinement();
}

/** \brief Messages callback. This function is called when syncronized pose and pointcloud
  * messages are received.
  * @return 
  * \param points_msg pointcloud message of type sensor_msgs::PointCloud2
  * \param pose_msg pose message of type geometry_msgs::PoseStamped
  */
void stereo_slam::IcpRefinementBase::msgsCallback(
                                  const sensor_msgs::PointCloud2ConstPtr& points_msg,
                                  const geometry_msgs::PoseStampedConstPtr& pose_msg)
{
}

/** \brief Reads the icp refinement node parameters
  * @return
  */
void stereo_slam::IcpRefinementBase::readParameters()
{
  Params icp_refinement_params;

  // Topic parameters
  nh_private_.getParam("queue_size", icp_refinement_params.queue_size);

  // Set the params
  setParams(icp_refinement_params);

  // Topics subscriptions
  std::string points_topic, pose_topic;
  nh_private_.param("points_topic", points_topic, std::string("/points2"));
  nh_private_.param("pose_topic", pose_topic, std::string("/slam_pose"));
  points_sub_.subscribe(nh_, points_topic, 1);
  pose_sub_  .subscribe(nh_, pose_topic, 1);
}

/** \brief Initializates the icp refinement node
  * @return
  */
bool stereo_slam::IcpRefinementBase::initializeIcpRefinement()
{
  // Callback syncronization
  approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(params_.queue_size),
                                  points_sub_, 
                                  pose_sub_) );
  approximate_sync_->registerCallback(boost::bind(
      &stereo_slam::IcpRefinementBase::msgsCallback,
      this, _1, _2));
}