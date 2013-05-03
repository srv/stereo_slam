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
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <Eigen/Geometry>
#include "postgresql_interface.h"
#include "utils.h"

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

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
    tf::Transform previous_tf_;
    double max_displacement_;
    bool first_message_;
    bool first_node_;
    int queue_size_;
    int node_counter_;
    boost::shared_ptr<database_interface::PostgresqlDatabase> pg_db_ptr_;
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
    g2o::SparseOptimizer graph_optimizer_;
    std::vector<g2o::VertexSE3*> graph_nodes_;
};

/** \brief Class constructor. Reads node parameters and initialize some properties.
  * @return 
  * \param nh public node handler
  * \param nhp private node handler
  */
stereo_localization::GraphConstructor::GraphConstructor(ros::NodeHandle nh, ros::NodeHandle nhp) : 
nh_(nh), nh_private_(nhp), first_message_(true), first_node_(true), node_counter_(0)
{

  // Database parameters
  nh_private_.param<std::string>("db_host", db_host_, "localhost");
  nh_private_.param<std::string>("db_port", db_port_, "5432");
  nh_private_.param<std::string>("db_user", db_user_, "postgres");
  nh_private_.param<std::string>("db_pass", db_pass_, "postgres");
  nh_private_.param<std::string>("db_name", db_name_, "graph");

  // Functional parameters
  nh_private_.param("max_displacement", max_displacement_, 0.2);
  nh_private_.param("queue_size", queue_size_, 5);

  // Topic parameters
  std::string odom_topic, image_topic;
  nh_private_.param("odom_topic", odom_topic, std::string("/odometry"));
  nh_private_.param("image_topic", image_topic, std::string("/left/image_rect_color"));

  // Topics subscriptions
  image_transport::ImageTransport it(nh_);
  odom_sub_ .subscribe(nh_, odom_topic, 1);
  image_sub_.subscribe(it, image_topic, 1);
  exact_sync_.reset(new ExactSync(ExactPolicy(queue_size_), odom_sub_, image_sub_) );
  exact_sync_->registerCallback(boost::bind(
                  &stereo_localization::GraphConstructor::msgsCallback, 
                  this, _1, _2));

  // Initialize the g2o graph optimizer
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  g2o::OptimizationAlgorithmGaussNewton* solverGauss = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
  graph_optimizer_.setAlgorithm(solverGauss);

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

  // Get the current odometry for these images
  tf::Vector3 tf_trans( odom_msg->pose.pose.position.x,
                        odom_msg->pose.pose.position.y,
                        odom_msg->pose.pose.position.z);
  tf::Quaternion tf_q(  odom_msg->pose.pose.orientation.x,
                        odom_msg->pose.pose.orientation.y,
                        odom_msg->pose.pose.orientation.z,
                        odom_msg->pose.pose.orientation.w);
  tf::Transform current_tf(tf_q, tf_trans);

  // First message
  if (first_message_)
  {
    // Save the transform
    previous_tf_ = current_tf;
    first_message_ = false;
  }
  else
  {
    // Check if difference between images is larger than maximum displacement
    tf::Vector3 d = current_tf.getOrigin() - previous_tf_.getOrigin();
    double norm = sqrt(d.x()*d.x() + d.y()*d.y() + d.z()*d.z());
    if (norm > max_displacement_)
    {
      // The image properties and pose must be saved into the database
      
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
      cv::Mat descriptors = cv::Mat_<std::vector<float> >();
      stereo_localization::Utils::keypointDetector(cv_ptr->image, key_points);
      stereo_localization::Utils::descriptorExtraction(cv_ptr->image, key_points, descriptors);
      ROS_INFO_STREAM("[GraphConstructor:] Descriptor size: " << descriptors.rows << ", " << descriptors.cols);

      // Transform descriptors to std::vector
      std::vector< std::vector<float> > descriptors_std = 
      stereo_localization::Utils::cvMatToStdMatrix(descriptors);

      // Save node descriptors into the database
      stereo_localization::GraphNodes node_data;
      node_data.descriptors_.data() = descriptors_std;
      if (!pg_db_ptr_->insertIntoDatabase(&node_data))
      {
        ROS_ERROR("[GraphConstructor:] Node insertion failed");
      }
      else
      {
        // Everything is ok, save the node into the graph

        // Build the pose
        Eigen::Vector3d t(    odom_msg->pose.pose.position.x,
                              odom_msg->pose.pose.position.y,
                              odom_msg->pose.pose.position.z);
        Eigen::Quaterniond q( odom_msg->pose.pose.orientation.x,
                              odom_msg->pose.pose.orientation.y,
                              odom_msg->pose.pose.orientation.z,
                              odom_msg->pose.pose.orientation.w);
        g2o::SE3Quat pose(q,t);

        // Build the node
        g2o::VertexSE3* v_se3 = new g2o::VertexSE3();
        v_se3->setId(node_data.id_.data());
        v_se3->setEstimate(pose);
        if (first_node_)
        {
          // First time, no edges.
          v_se3->setFixed(true);
          graph_optimizer_.addVertex(v_se3);
          first_node_ = false;
        }
        else
        {
          // When graph has been initialized get the transform between current and previous pose
          // and save it as an edge
          tf::Transform tfCurrentToPrevious = current_tf.inverseTimes(previous_tf_);
          tf::Vector3 tCurrentToPrevious = tfCurrentToPrevious.getOrigin();
          tf::Quaternion qCurrentToPrevious = tfCurrentToPrevious.getRotation();
          Eigen::Vector3d t_e(    tCurrentToPrevious.x(),
                                  tCurrentToPrevious.y(),
                                  tCurrentToPrevious.z());
          Eigen::Quaterniond q_e( qCurrentToPrevious.x(),
                                  qCurrentToPrevious.y(),
                                  qCurrentToPrevious.z(),
                                  qCurrentToPrevious.w());
          g2o::SE3Quat transform(q_e,t_e);

          // Create new edge
          g2o::EdgeSE3* edge = new g2o::EdgeSE3();
          edge->setVertex(0, v_se3);
          edge->setVertex(1, graph_nodes_.at(node_counter_));
          edge->setMeasurement(transform);
          graph_optimizer_.addEdge(edge);
        }
        
        // Save the node into vector
        graph_nodes_.push_back(v_se3);
        node_counter_++;

        // Save the transform
        previous_tf_ = current_tf;

        ROS_INFO_STREAM("[GraphConstructor:] Node " << node_data.id_.data() << " insertion succeeded");
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
