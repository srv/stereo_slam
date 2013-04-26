#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "utils.h"

namespace stereo_localization
{

class MatchingProcess {

  public:
    MatchingProcess(ros::NodeHandle nh, ros::NodeHandle nhp);
  protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
};

} // Namespace

stereo_localization::MatchingProcess::MatchingProcess(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nh_private_(nhp)
{
}
