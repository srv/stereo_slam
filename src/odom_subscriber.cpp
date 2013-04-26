#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class OdomSubscriber{

  public:
    std::string topic;
    double max_displacement;
    OdomSubscriber(ros::NodeHandle nh, ros::NodeHandle nhp);
  protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
  private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    tf::Vector3 position_;
    bool first_message_;
};

OdomSubscriber::OdomSubscriber(ros::NodeHandle nh, ros::NodeHandle nhp) : nh_(nh), nh_private_(nhp), first_message_(true)
{

  nh_private_.param<std::string>("odometry_topic", topic, "/odom");
  nh_private_.param("max_displacement", max_displacement, 2.0);

  ros::Subscriber sub = nh_.subscribe<nav_msgs::Odometry>(topic, 10, &OdomSubscriber::odomCallback, this);
  if (sub == ros::Subscriber() )
  {
    ROS_ERROR("Cannot subscribe to topic %s", topic.c_str());
  }
}

void OdomSubscriber::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  tf::Vector3 actual_position;

  actual_position = tf::Vector3(msg->pose.pose.position.x,
                                msg->pose.pose.position.y,
                                msg->pose.pose.position.z);

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
