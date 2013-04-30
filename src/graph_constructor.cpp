#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <database_interface/postgresql_database.h>
#include <boost/shared_ptr.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <image_transport/subscriber_filter.h>
#include <opencv2/features2d/features2d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <libpq-fe.h>
#include "postgresql_interface.h"
#include "utils.h"

namespace enc = sensor_msgs::image_encodings;

namespace stereo_localization
{

class GraphConstructor
{
  public:
    GraphConstructor(ros::NodeHandle nh, ros::NodeHandle nhp);
  protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
  private:
    void msgsCallback(const nav_msgs::Odometry::ConstPtr& msg,
                      const sensor_msgs::ImageConstPtr& image_msg);
    tf::Vector3 previous_position_;
    double max_displacement_;
    bool first_message_;
    int queue_size_;
    boost::shared_ptr<database_interface::PostgresqlDatabase> pg_db_ptr_;
    std::vector< boost::shared_ptr<GraphNodes> > map_nodes_;
    std::string db_host_;
    std::string db_port_;
    std::string db_user_;
    std::string db_pass_;
    std::string db_name_;
    typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, sensor_msgs::Image> ExactPolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    boost::shared_ptr<ExactSync> exact_sync_;
    image_transport::SubscriberFilter image_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    PGconn* connection_init_;
};

/** \brief Class constructor. Reads node parameters and initialize some properties.
  * @return 
  * \param nh public node handler
  * \param nhp private node handler
  */
stereo_localization::GraphConstructor::GraphConstructor(ros::NodeHandle nh, ros::NodeHandle nhp) : 
nh_(nh), nh_private_(nhp), first_message_(true)
{

  // Database parameters
  nh_private_.param<std::string>("db_host", db_host_, "localhost");
  nh_private_.param<std::string>("db_port", db_port_, "5432");
  nh_private_.param<std::string>("db_user", db_user_, "postgres");
  nh_private_.param<std::string>("db_pass", db_pass_, "postgres");
  nh_private_.param<std::string>("db_name", db_name_, "graph");

  // Functional parameters
  nh_private_.param("max_displacement", max_displacement_, 0.5);
  nh_private_.param("queue_size", queue_size_, 5);

  // Topic parameters
  std::string image_topic, odom_topic;
  nh_private_.param("image_topic", image_topic, std::string("/left/image_rect_color"));
  nh_private_.param("odom_topic", odom_topic, std::string("/odometry"));

  // Topics subscriptions
  image_transport::ImageTransport it(nh_);
  odom_sub_ .subscribe(nh_, odom_topic, 1);
  image_sub_.subscribe(it, image_topic, 1);
  exact_sync_.reset(new ExactSync(ExactPolicy(queue_size_), odom_sub_, image_sub_) );
  exact_sync_->registerCallback(boost::bind(
                  &stereo_localization::GraphConstructor::msgsCallback, 
                  this, _1, _2));

  // Database initialization
  boost::shared_ptr<database_interface::PostgresqlDatabase> db_ptr( 
    new database_interface::PostgresqlDatabase(db_host_, db_port_, db_user_, db_pass_, db_name_));
  pg_db_ptr_ = db_ptr;

  if (!pg_db_ptr_->isConnected())
  {
    ROS_ERROR("[GraphConstructor:] Database failed to connect");
  }
  else
  {
    ROS_INFO("[GraphConstructor:] Database connected successfully!");

    // Database table creation. New connection is needed due to the interface design
    std::string conn_info = "host=" + db_host_ + " port=" + db_port_ + 
      " user=" + db_user_ + " password=" + db_pass_ + " dbname=" + db_name_;
    connection_init_= PQconnectdb(conn_info.c_str());
    if (PQstatus(connection_init_)!=CONNECTION_OK) 
    {
      ROS_ERROR("Database connection failed with error message: %s", PQerrorMessage(connection_init_));
    }
    else
    {
      // Create the table (if no exists)
      std::string query_create("CREATE TABLE IF NOT EXISTS graph_nodes"
                        "( "
                          "id bigserial primary key, "
                          "pose_x double precision NOT NULL, "
                          "pose_y double precision NOT NULL, "
                          "pose_z double precision NOT NULL, "
                          "pose_rotation double precision[4] NOT NULL, "
                          "descriptors double precision[][] "
                        ")");
      PQexec(connection_init_, query_create.c_str());
      
      // Delete all rows of the new table (to start clean)
      std::string query_delete("DELETE FROM graph_nodes");
      PQexec(connection_init_, query_delete.c_str());
      ROS_INFO("[GraphConstructor:] graph_nodes table created successfully!");
    }
  }
}

/** \brief Messages callback. This function is called when syncronized odometry and image
  * message are received.
  * @return 
  * \param odom_msg ros odometry message of type nav_msgs::Odometry
  * \param image_msg ros image message of type sensor_msgs::Image
  */
void stereo_localization::GraphConstructor::msgsCallback(
                                  const nav_msgs::Odometry::ConstPtr& odom_msg,
                                  const sensor_msgs::ImageConstPtr& image_msg)
{

  // Get the current odometry for this images
  tf::Vector3 current_position = tf::Vector3( odom_msg->pose.pose.position.x,
                                              odom_msg->pose.pose.position.y,
                                              odom_msg->pose.pose.position.z);

  // First message
  if(first_message_)
  {
    previous_position_ = current_position;
    first_message_ = false;
  }
  else
  {
    // Check if difference between images is larger than maximum displacement
    tf::Vector3 d = current_position - previous_position_;
    double norm = sqrt(d.x()*d.x() + d.y()*d.y() + d.z()*d.z());
    if(norm > max_displacement_)
    {
      // The image properties and pose must be saved into the database
      ROS_INFO_STREAM("[GraphConstructor:] Odometry threshold passed. Displacement from previous is " << norm);
      
      // Convert message to cv::Mat
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(image_msg, enc::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("[GraphConstructor:] cv_bridge exception: %s", e.what());
        return;
      }

      // Extract keypoints and descriptors of image
      std::vector<cv::KeyPoint> key_points;
      cv::Mat descriptors;
      stereo_localization::Utils::keypointDetector(cv_ptr->image, key_points);
      stereo_localization::Utils::descriptorExtraction(cv_ptr->image, key_points, descriptors);
      ROS_INFO_STREAM("[GraphConstructor:] Descriptor size: " << descriptors.rows << ", " << descriptors.cols);

      // Round and transform descriptors to std::vector
      std::vector< std::vector<double> > descriptors_std = stereo_localization::Utils::matrixRound(descriptors);

      // Save node into the database
      std::vector<double> rot;
      rot.push_back(odom_msg->pose.pose.orientation.x);
      rot.push_back(odom_msg->pose.pose.orientation.y);
      rot.push_back(odom_msg->pose.pose.orientation.z);
      rot.push_back(odom_msg->pose.pose.orientation.w);
      stereo_localization::GraphNodes new_node;
      new_node.pose_x_.data() = current_position.x();
      new_node.pose_y_.data() = current_position.y();
      new_node.pose_z_.data() = current_position.z();
      new_node.pose_rotation_.data() = rot;
      new_node.descriptors_.data() = descriptors_std;

      if (!pg_db_ptr_->insertIntoDatabase(&new_node))
      {
        ROS_ERROR("[GraphConstructor:] Node insertion failed");
      }
      else
      {
        // Save previous
        previous_position_ = current_position;
        ROS_INFO("[GraphConstructor:] Node insertion succeeded");
      }
    }
  }
}

} // Namespace


int main(int argc, char** argv)
{
  ros::init(argc,argv,"graph_constructor");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  stereo_localization::GraphConstructor graphConstructor(nh,nh_private);

  ros::spin();

  return 0;
}
