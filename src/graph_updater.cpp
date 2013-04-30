#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <database_interface/postgresql_database.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/subscriber_filter.h>
#include "postgresql_interface.h"
#include "utils.h"

namespace stereo_localization
{

class GraphUpdater {

  public:
    GraphUpdater(ros::NodeHandle nh, ros::NodeHandle nhp);
  protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
  private:
    void msgsCallback(const nav_msgs::Odometry::ConstPtr& msg,
                      const sensor_msgs::ImageConstPtr& left_msg,
                      const sensor_msgs::ImageConstPtr& right_msg,
                      const sensor_msgs::CameraInfoConstPtr& left_info_msg,
                      const sensor_msgs::CameraInfoConstPtr& right_info_msg);
    int queue_size_;
    typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactPolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    boost::shared_ptr<ExactSync> exact_sync_;
    image_transport::SubscriberFilter left_sub_, right_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_, right_info_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
};

/** \brief Class constructor. Reads node parameters and initialize some properties.
  * @return 
  * \param nh public node handler
  * \param nhp private node handler
  */
stereo_localization::GraphUpdater::GraphUpdater(ros::NodeHandle nh, ros::NodeHandle nhp): 
nh_(nh), nh_private_(nhp)
{
  // Resolve topics regarding stereo camera and image
	std::string stereo_ns = nh_.resolveName("stereo");
	std::string left_topic = ros::names::clean(stereo_ns + "/left/" + nh_.resolveName("image"));
	std::string right_topic = ros::names::clean(stereo_ns + "/right/" + nh_.resolveName("image"));

	std::string left_info_topic = stereo_ns + "/left/camera_info";
	std::string right_info_topic = stereo_ns + "/right/camera_info";

	std::string odom_topic = nh_.resolveName("odom");

  // Get the queue size param
	nh_private_.param("queue_size", queue_size_, 5);

	// Subscribe to five input topics.
	ROS_INFO("Subscribing to:\n\t* %s\n\t* %s\n\t* %s\n\t* %s\n\t* %s", 
	    odom_topic.c_str(),
	    left_topic.c_str(), right_topic.c_str(),
	    left_info_topic.c_str(), right_info_topic.c_str());

	image_transport::ImageTransport it(nh_);
  odom_sub_ .subscribe(nh_, odom_topic, 1);
  left_sub_ .subscribe(it, left_topic, 1);
  right_sub_.subscribe(it, right_topic, 1);
  left_info_sub_.subscribe(nh_, left_info_topic, 1);
  right_info_sub_.subscribe(nh_, right_info_topic, 1);

  exact_sync_.reset(new ExactSync(ExactPolicy(queue_size_),
                  odom_sub_, left_sub_, right_sub_, left_info_sub_, right_info_sub_) );

  // Set the callback
  exact_sync_->registerCallback(boost::bind(&stereo_localization::GraphUpdater::msgsCallback, 
                  this, _1, _2, _3, _4, _5));
}

/** \brief Messages callback. This function is called when syncronized odometry and image
  * message are received.
  * @return 
  * \param odom_msg ros odometry message of type nav_msgs::Odometry
  * \param left_msg ros left image message of type sensor_msgs::Image
  * \param right_msg ros right image message of type sensor_msgs::Image
  * \param left_info_msg ros left camera info message of type sensor_msgs::Image
  * \param right_info_msg ros right camera info message of type sensor_msgs::Image
  */
void stereo_localization::GraphUpdater::msgsCallback(
																	const nav_msgs::Odometry::ConstPtr& odom_msg,
                                  const sensor_msgs::ImageConstPtr& left_msg,
                                  const sensor_msgs::ImageConstPtr& right_msg,
                                  const sensor_msgs::CameraInfoConstPtr& left_info_msg,
                                  const sensor_msgs::CameraInfoConstPtr& right_info_msg)
{
  
}

} // Namespace

int main(int argc, char** argv)
{
  ros::init(argc,argv,"graph_updater");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  stereo_localization::GraphUpdater graphUpdater(nh,nh_private);

  ros::spin();

  return 0;
}
