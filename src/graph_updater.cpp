#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <database_interface/postgresql_database.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "postgresql_interface.h"
#include "utils.h"

namespace enc = sensor_msgs::image_encodings;

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
    double candidate_threshold_;
    double matching_threshold_;
    boost::shared_ptr<database_interface::PostgresqlDatabase> pg_db_ptr_;
    std::string db_host_;
    std::string db_port_;
    std::string db_user_;
    std::string db_pass_;
    std::string db_name_;
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
  // Database parameters
  nh_private_.param<std::string>("db_host", db_host_, "localhost");
  nh_private_.param<std::string>("db_port", db_port_, "5432");
  nh_private_.param<std::string>("db_user", db_user_, "postgres");
  nh_private_.param<std::string>("db_pass", db_pass_, "postgres");
  nh_private_.param<std::string>("db_name", db_name_, "graph");

  // Functional parameters
  nh_private_.param("candidate_threshold", candidate_threshold_, 0.5);
  nh_private_.param("matching_threshold", matching_threshold_, 0.8);
  nh_private_.param("queue_size", queue_size_, 5);

  // Topic parameters
  std::string odom_topic, left_topic, right_topic, left_info_topic, right_info_topic;
  nh_private_.param("odom_topic", odom_topic, std::string("/odometry"));
  nh_private_.param("left_topic", left_topic, std::string("/left/image_rect_color"));
  nh_private_.param("right_topic", right_topic, std::string("/right/image_rect_color"));
  nh_private_.param("left_info_topic", left_info_topic, std::string("/left/camera_info"));
  nh_private_.param("right_info_topic", right_info_topic, std::string("/right/camera_info"));

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

  // Database initialization
  boost::shared_ptr<database_interface::PostgresqlDatabase> db_ptr( 
    new database_interface::PostgresqlDatabase(db_host_, db_port_, db_user_, db_pass_, db_name_));
  pg_db_ptr_ = db_ptr;

  if (!pg_db_ptr_->isConnected())
  {
    ROS_ERROR("[GraphUpdater:] Database failed to connect");
  }
  else
  {
    ROS_INFO("[GraphUpdater:] Database connected successfully!");
  }
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
  // Get the current odometry for these images
  tf::Vector3 current_position = tf::Vector3( odom_msg->pose.pose.position.x,
                                              odom_msg->pose.pose.position.y,
                                              odom_msg->pose.pose.position.z);

  // Get candidates for matching
  std::string min_x = boost::lexical_cast<std::string>(current_position.x() - candidate_threshold_);
  std::string max_x = boost::lexical_cast<std::string>(current_position.x() + candidate_threshold_);
  std::string min_y = boost::lexical_cast<std::string>(current_position.y() - candidate_threshold_);
  std::string max_y = boost::lexical_cast<std::string>(current_position.y() + candidate_threshold_);
  std::string min_z = boost::lexical_cast<std::string>(current_position.z() - candidate_threshold_);
  std::string max_z = boost::lexical_cast<std::string>(current_position.z() + candidate_threshold_);

  std::vector< boost::shared_ptr<stereo_localization::GraphNodes> > candidate_nodes;
  std::string where_clause = "(pose_x BETWEEN " + min_x + " AND " + max_x + ") AND "
                             "(pose_y BETWEEN " + min_y + " AND " + max_y + ") AND "
                             "(pose_z BETWEEN " + min_z + " AND " + max_z + ")";
  pg_db_ptr_->getList(candidate_nodes, where_clause);

  if (candidate_nodes.size() > 0)
  {
    // Candidate nodes for matching have been found into the database
    ROS_INFO_STREAM("[GraphUpdater:] Candidates found: " << candidate_nodes.size());

    // Convert message to cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(left_msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("[GraphUpdater:] cv_bridge exception: %s", e.what());
      return;
    }

    // Extract keypoints and descriptors of image
    std::vector<cv::KeyPoint> key_points_current;
    cv::Mat desc_current = cv::Mat_<std::vector<float> >();
    stereo_localization::Utils::keypointDetector(cv_ptr->image, key_points_current);
    stereo_localization::Utils::descriptorExtraction(cv_ptr->image, key_points_current, desc_current);

    // Search descriptors correspondences between candidates
    for (unsigned int i=0; i<candidate_nodes.size(); i++)
    {
      // Get the descriptors of the candidate
      cv::Mat desc_candidate = cv::Mat_<std::vector<float> >();
      desc_candidate = stereo_localization::Utils::stdMatrixToCvMat(candidate_nodes[i]->descriptors_.data());

      // Compute matchings
      std::vector<cv::DMatch> matches;
      stereo_localization::Utils::thresholdMatching(desc_current, 
        desc_candidate, matches, matching_threshold_);

      ROS_INFO_STREAM("Num of matches: " << matches.size());

      // we need at least 5 matches for solvePnPRansac
      //if (matches.size() < 5)
    }
  }
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
