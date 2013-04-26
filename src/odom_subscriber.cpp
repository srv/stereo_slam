#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <database_interface/postgresql_database.h>
#include <boost/shared_ptr.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/subscriber_filter.h>

#include "map_node.h"

class OdomSubscriber{

  public:
    double max_displacement;
    OdomSubscriber(ros::NodeHandle nh, ros::NodeHandle nhp);
  protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
  private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg,
                                  const sensor_msgs::ImageConstPtr& left_msg,
                                  const sensor_msgs::CameraInfoConstPtr& left_info_msg,
                                  const sensor_msgs::ImageConstPtr& right_msg,
                                  const sensor_msgs::CameraInfoConstPtr& right_info_msg);
    tf::Vector3 position_;
    bool first_message_;
    boost::shared_ptr<database_interface::PostgresqlDatabase> map_db_ptr_;
    std::vector< boost::shared_ptr<MapNode> > map_nodes_;
    std::string db_host_;
    std::string db_port_;
    std::string db_user_;
    std::string db_pass_;
    std::string db_name_;
    image_transport::SubscriberFilter left_sub_, right_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_, right_info_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
};

OdomSubscriber::OdomSubscriber(ros::NodeHandle nh, ros::NodeHandle nhp) : nh_(nh), nh_private_(nhp), first_message_(true)
{

  nh_private_.param("max_displacement", max_displacement, 2.0);
  nh_private_.param<std::string>("db_host", db_host_, "localhost");
  nh_private_.param<std::string>("db_port", db_port_, "5432");
  nh_private_.param<std::string>("db_user", db_user_, "postgres");
  nh_private_.param<std::string>("db_pass", db_pass_, "postgres");
  nh_private_.param<std::string>("db_name", db_name_, "map");

  std::string stereo_ns = nh_.resolveName("stereo");
  std::string left_topic = ros::names::clean(stereo_ns + "/left/" + nh_.resolveName("image"));
  std::string right_topic = ros::names::clean(stereo_ns + "/right/" + nh_.resolveName("image"));

  std::string left_info_topic = stereo_ns + "/left/camera_info";
  std::string right_info_topic = stereo_ns + "/right/camera_info";

  std::string odom_topic = nh_.resolveName("odom");

  // Subscribe to five input topics.
  ROS_INFO("Subscribing to:\n\t* %s\n\t* %s\n\t* %s\n\t* %s\n\t* %s", 
      odom_topic.c_str(),
      left_topic.c_str(), right_topic.c_str(),
      left_info_topic.c_str(), right_info_topic.c_str());

  odom_sub_ .subscribe(nh_,odom_topic, 1);
  image_transport::ImageTransport it(nh_);
  left_sub_ .subscribe(it,left_topic, 1);
  right_sub_.subscribe(it,right_topic, 1);
  left_info_sub_.subscribe(nh_, left_info_topic, 1);
  left_info_sub_.subscribe(nh_, right_info_topic, 1);

  message_filters::TimeSynchronizer< nav_msgs::Odometry,
                  sensor_msgs::Image, 
                  sensor_msgs::CameraInfo, 
                  sensor_msgs::Image, 
                  sensor_msgs::CameraInfo> sync(odom_sub_, left_sub_, left_info_sub_, right_sub_, right_info_sub_, 3);
  sync.registerCallback(boost::bind(&OdomSubscriber::odomCallback, this, _1, _2, _3, _4, _5));



  boost::shared_ptr<database_interface::PostgresqlDatabase> db_ptr( new database_interface::PostgresqlDatabase(db_host_, db_port_, db_user_, db_pass_, db_name_));
  
  map_db_ptr_ = db_ptr;

  if (!map_db_ptr_->isConnected())
  {
    ROS_ERROR("Database failed to connect");
  }
  ROS_INFO("Database connected successfully!");
}

void OdomSubscriber::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,
                                  const sensor_msgs::ImageConstPtr& left_msg,
                                  const sensor_msgs::CameraInfoConstPtr& left_info_msg,
                                  const sensor_msgs::ImageConstPtr& right_msg,
                                  const sensor_msgs::CameraInfoConstPtr& right_info_msg)
{
  tf::Vector3 actual_position;

  actual_position = tf::Vector3(odom_msg->pose.pose.position.x,
                                odom_msg->pose.pose.position.y,
                                odom_msg->pose.pose.position.z);

  if(first_message_)
  {
    position_ = actual_position;
    first_message_ = false;
  }else{
    tf::Vector3 d = actual_position - position_;
    double norm = sqrt(d.x()*d.x() + d.y()*d.y() + d.z()*d.z());
    if(norm > max_displacement){
      // Fire the required events
      ROS_INFO(" Max displacement ");
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"odom_to_tf_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  OdomSubscriber odomSubscriber(nh,nh_private);

  ros::spin();

  return 0;
}
