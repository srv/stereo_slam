#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <database_interface/postgresql_database.h>
#include <boost/shared_ptr.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <image_transport/subscriber_filter.h>
#include <opencv2/features2d/features2d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/stereo_camera_model.h>
#include <libpq-fe.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
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
    void timerCallback(const ros::WallTimerEvent& event);
  private:
    void msgsCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,
                      const sensor_msgs::ImageConstPtr& l_img,
                      const sensor_msgs::ImageConstPtr& r_img,
                      const sensor_msgs::CameraInfoConstPtr& l_info,
                      const sensor_msgs::CameraInfoConstPtr& r_info);
    tf::Transform previous_pose_;
    double update_rate_;
    double max_displacement_;
    double candidate_threshold_;
    double descriptors_threshold_;
    int matches_threshold_;
    bool first_message_;
    bool first_node_;
    bool block_update_;
    bool go2_verbose_;
    int queue_size_;
    int ids_;
    int last_pub_pose_;
    int go2_opt_max_iter_;
    image_geometry::StereoCameraModel stereo_camera_model_;
    cv::Mat camera_matrix_;
    boost::shared_ptr<database_interface::PostgresqlDatabase> pg_db_ptr_;
    std::string db_host_;
    std::string db_port_;
    std::string db_user_;
    std::string db_pass_;
    std::string db_name_;
    typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, 
                                                      sensor_msgs::Image, 
                                                      sensor_msgs::Image, 
                                                      sensor_msgs::CameraInfo, 
                                                      sensor_msgs::CameraInfo> ExactPolicy;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, 
                                                      sensor_msgs::Image, 
                                                      sensor_msgs::Image, 
                                                      sensor_msgs::CameraInfo, 
                                                      sensor_msgs::CameraInfo> ApproximatePolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    boost::shared_ptr<ExactSync> exact_sync_;
    boost::shared_ptr<ApproximateSync> approximate_sync_;
    image_transport::SubscriberFilter left_sub_, right_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_, right_info_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    ros::Publisher pose_pub_;
    std::string map_frame_id_;
    PGconn* connection_init_;
    g2o::SparseOptimizer graph_optimizer_;
    g2o::VertexSE3* prev_node_;
    ros::WallTimer timer_;
    std::vector<cv::Point2i> false_candidates_;
};

/** \brief Class constructor. Reads node parameters and initialize some properties.
  * @return 
  * \param nh public node handler
  * \param nhp private node handler
  */
stereo_localization::GraphConstructor::GraphConstructor(ros::NodeHandle nh, ros::NodeHandle nhp) : 
nh_(nh), nh_private_(nhp)
{

  // Initializations
  first_message_ = true;
  first_node_ = true;
  block_update_ = false;
  ids_ = 0;
  last_pub_pose_ = -1;

  // Database parameters
  nh_private_.param<std::string>("db_host", db_host_, "localhost");
  nh_private_.param<std::string>("db_port", db_port_, "5432");
  nh_private_.param<std::string>("db_user", db_user_, "postgres");
  nh_private_.param<std::string>("db_pass", db_pass_, "postgres");
  nh_private_.param<std::string>("db_name", db_name_, "graph");

  // Functional parameters
  nh_private_.param("update_rate", update_rate_, 0.5);
  nh_private_.param("max_displacement", max_displacement_, 0.5);
  nh_private_.param("candidate_threshold", candidate_threshold_, 0.51);
  nh_private_.param("descriptors_threshold", descriptors_threshold_, 0.8);
  nh_private_.param("matches_threshold", matches_threshold_, 70);

  // G2O parameters
  nh_private_.param("go2_verbose", go2_verbose_, false);
  nh_private_.param("go2_opt_max_iter", go2_opt_max_iter_, 10);

  // Topic parameters
  std::string odom_topic, left_topic, right_topic, left_info_topic, right_info_topic;
  nh_private_.param("queue_size", queue_size_, 5);
  nh_private_.param("odom_topic", odom_topic, std::string("/odometry"));
  nh_private_.param("left_topic", left_topic, std::string("/left/image_rect_color"));
  nh_private_.param("right_topic", right_topic, std::string("/right/image_rect_color"));
  nh_private_.param("left_info_topic", left_info_topic, std::string("/left/camera_info"));
  nh_private_.param("right_info_topic", right_info_topic, std::string("/right/camera_info"));
  nh_private_.param("map_frame_id", map_frame_id_, std::string("/map"));

  // Topics subscriptions
  image_transport::ImageTransport it(nh_);
  odom_sub_ .subscribe(nh_, odom_topic, 1);
  left_sub_ .subscribe(it, left_topic, 1);
  right_sub_.subscribe(it, right_topic, 1);
  left_info_sub_.subscribe(nh_, left_info_topic, 1);
  right_info_sub_.subscribe(nh_, right_info_topic, 1);

  // Callback syncronization
  bool approx;
  nh_private_.param("approximate_sync", approx, false);
  if (approx)
  {
    approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size_),
                                                odom_sub_, left_sub_, right_sub_, left_info_sub_, right_info_sub_) );
    approximate_sync_->registerCallback(boost::bind(&stereo_localization::GraphConstructor::msgsCallback,
                    this, _1, _2, _3, _4, _5));
  }
  else
  {
    exact_sync_.reset(new ExactSync(ExactPolicy(queue_size_),
                    odom_sub_, left_sub_, right_sub_, left_info_sub_, right_info_sub_) );
    exact_sync_->registerCallback(boost::bind(&stereo_localization::GraphConstructor::msgsCallback, 
                    this, _1, _2, _3, _4, _5));
  }

  // Advertise topics
  pose_pub_ = nh_private_.advertise<geometry_msgs::PoseStamped>("pose", 1);

  // Initialize the g2o graph optimizer
  /*
  SlamLinearSolver* linear_solver_ptr = new SlamLinearSolver();
  linear_solver_ptr->setBlockOrdering(false);
  SlamBlockSolver* block_solver_ptr = new SlamBlockSolver(linear_solver_ptr);
  g2o::OptimizationAlgorithmGaussNewton* solver_gauss_ptr = 
  new g2o::OptimizationAlgorithmGaussNewton(block_solver_ptr);
  graph_optimizer_.setAlgorithm(solver_gauss_ptr);
  */
  g2o::BlockSolverX::LinearSolverType * linear_solver_ptr;
  linear_solver_ptr = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();
  g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linear_solver_ptr);
  g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  graph_optimizer_.setAlgorithm(solver);

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
      // Drop the table (to start clean)
      std::string query_delete("DROP TABLE IF EXISTS graph_nodes");
      PQexec(connection_init_, query_delete.c_str());
      ROS_INFO("[GraphConstructor:] graph_nodes table droped successfully!");

      // Create the table (if no exists)
      std::string query_create("CREATE TABLE IF NOT EXISTS graph_nodes"
                        "( "
                          "id bigserial primary key, "
                          "keypoints double precision[][], "
                          "descriptors double precision[][], "
                          "points3d double precision[][] "
                        ")");
      PQexec(connection_init_, query_create.c_str());
      ROS_INFO("[GraphConstructor:] graph_nodes table created successfully!");
    }
  }

  // Start timer for graph update
  timer_ = nh.createWallTimer(ros::WallDuration(update_rate_), 
                            &stereo_localization::GraphConstructor::timerCallback,
                            this);

  // Parameters check
  if (matches_threshold_ < 5)
  {
    ROS_WARN("Parameter 'matches_threshold' must be greater than 5. Set to 6.");
    matches_threshold_ = 6;
  }
}

/** \brief Messages callback. This function is called when syncronized odometry and image
  * message are received.
  * @return 
  * \param odom_msg ros odometry message of type nav_msgs::Odometry
  * \param left_msg ros image message of type sensor_msgs::Image
  */
void stereo_localization::GraphConstructor::msgsCallback(
                                  const nav_msgs::Odometry::ConstPtr& odom_msg,
                                  const sensor_msgs::ImageConstPtr& l_img,
                                  const sensor_msgs::ImageConstPtr& r_img,
                                  const sensor_msgs::CameraInfoConstPtr& l_info,
                                  const sensor_msgs::CameraInfoConstPtr& r_info)
{
  // Set camera info
  stereo_camera_model_.fromCameraInfo(l_info, r_info);
  const cv::Mat P(3,4, CV_64FC1, const_cast<double*>(l_info->P.data()));
  camera_matrix_ = P.colRange(cv::Range(0,3)).clone();

  // Get the current odometry for these images
  tf::Vector3 tf_trans( odom_msg->pose.pose.position.x,
                        odom_msg->pose.pose.position.y,
                        odom_msg->pose.pose.position.z);
  tf::Quaternion tf_q ( odom_msg->pose.pose.orientation.x,
                        odom_msg->pose.pose.orientation.y,
                        odom_msg->pose.pose.orientation.z,
                        odom_msg->pose.pose.orientation.w);
  tf::Transform current_pose(tf_q, tf_trans);

  // First message
  if (first_message_)
  {
    // Save the transform
    previous_pose_ = current_pose;
    first_message_ = false;
  }
  else
  {
    // Check if difference between images is larger than maximum displacement
    if (stereo_localization::Utils::poseDiff(current_pose, previous_pose_) > max_displacement_)
    {
      // The image properties and pose must be saved into the database
      
      // Convert message to cv::Mat
      cv_bridge::CvImagePtr l_ptr, r_ptr;
      try
      {
        l_ptr = cv_bridge::toCvCopy(l_img, enc::BGR8);
        r_ptr = cv_bridge::toCvCopy(r_img, enc::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("[GraphConstructor:] cv_bridge exception: %s", e.what());
        return;
      }

      // Extract keypoints and descriptors of images
      std::vector<cv::KeyPoint> l_kp, r_kp;
      cv::Mat l_desc = cv::Mat_<std::vector<float> >();
      cv::Mat r_desc = cv::Mat_<std::vector<float> >();
      stereo_localization::Utils::keypointDetector(l_ptr->image, l_kp);
      stereo_localization::Utils::keypointDetector(r_ptr->image, r_kp);
      stereo_localization::Utils::descriptorExtraction(l_ptr->image, l_kp, l_desc);
      stereo_localization::Utils::descriptorExtraction(r_ptr->image, r_kp, r_desc);

      // Find matching between stereo images
      std::vector<cv::DMatch> matches;
      stereo_localization::Utils::thresholdMatching(l_desc, r_desc, matches, descriptors_threshold_);

      // Compute 3D points
      std::vector<cv::Point2f> matched_keypoints;
      std::vector<cv::Point3f> matched_3d_points;
      cv::Mat matched_descriptors;
      for (size_t i = 0; i < matches.size(); ++i)
      {
        int index_left = matches[i].queryIdx;
        int index_right = matches[i].trainIdx;
        cv::Point3d world_point;
        stereo_localization::Utils::calculate3DPoint( stereo_camera_model_,
                                                      l_kp[index_left].pt,
                                                      r_kp[index_right].pt,
                                                      world_point);
        matched_3d_points.push_back(world_point);
        matched_keypoints.push_back(l_kp[index_left].pt);
        matched_descriptors.push_back(l_desc.row(index_left));
      }

      // Transform data to std::vector for database
      std::vector< std::vector<float> > keypoints = 
      stereo_localization::Utils::cvPoint2fToStdMatrix(matched_keypoints);
      std::vector< std::vector<float> > descriptors = 
      stereo_localization::Utils::cvMatToStdMatrix(matched_descriptors);
      std::vector< std::vector<float> > points_3d = 
      stereo_localization::Utils::cvPoint3fToStdMatrix(matched_3d_points);
      // Save node data into the database
      stereo_localization::GraphNodes node_data;
      node_data.keypoints_.data() = keypoints;
      node_data.descriptors_.data() = descriptors;
      node_data.points3d_.data() = points_3d;
      if (!pg_db_ptr_->insertIntoDatabase(&node_data))
      {
        ROS_ERROR("[GraphConstructor:] Node insertion failed");
      }
      else
      {
        // Everything is ok, save the node into the graph

        // Build the pose
        Eigen::Isometry3d pose = stereo_localization::Utils::tfToEigen(current_pose);

        // Build the node
        g2o::VertexSE3* cur_node = new g2o::VertexSE3();
        cur_node->setId(node_data.id_.data() - 1);
        cur_node->setEstimate(pose);
        if (first_node_)
        {
          // First time, no edges.
          cur_node->setFixed(true);
          graph_optimizer_.addVertex(cur_node);
          first_node_ = false;
        }
        else
        {
          // When graph has been initialized get the transform between current and previous nodes
          // and save it as an edge
          graph_optimizer_.addVertex(cur_node);

          // Odometry edges
          g2o::EdgeSE3* e = new g2o::EdgeSE3();
          Eigen::Isometry3d t = prev_node_->estimate().inverse() * cur_node->estimate();
          e->setVertex(0, prev_node_);
          e->setVertex(1, cur_node);
          e->setMeasurement(t);
          graph_optimizer_.addEdge(e);
        }

        // Save the transform and node
        prev_node_ = cur_node;
        previous_pose_ = current_pose;

        ROS_INFO_STREAM("[GraphConstructor:] Node " << node_data.id_.data() << " insertion succeeded");
      }
    }
  }
}

/** \brief Timer callback. This function is called when update timer time is ellapsed.
  * @return 
  * \param event is the timer event object
  */
void stereo_localization::GraphConstructor::timerCallback(const ros::WallTimerEvent& event)
{
  // Check if callback is currently executed
  if (block_update_)
    return;

  // Prevent for callback re-called
  block_update_ = true;

  // True when new edges to force optimization
  bool edge_added = false;

  // Find possible candidates for loop-closing
  for (unsigned int i=0; i<graph_optimizer_.vertices().size(); i++)
  {
    // Extract the pose of node i
    g2o::VertexSE3* v_i = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[i]);
    tf::Transform pose_i = stereo_localization::Utils::getNodePose(v_i);

    for (unsigned int j=i+1; j<graph_optimizer_.vertices().size(); j++)
    {
      // Extract the pose of node j and compare
      g2o::VertexSE3* v_j = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[j]);
      tf::Transform pose_j = stereo_localization::Utils::getNodePose(v_j);

      // Check if there is an edge that currently join both nodes
      bool edge_found = false;
      for (g2o::OptimizableGraph::EdgeSet::iterator it=graph_optimizer_.edges().begin();
       it!=graph_optimizer_.edges().end(); it++)
      {
        g2o::EdgeSE3* e = dynamic_cast<g2o::EdgeSE3*> (*it);
        if (e)
        {
          if (  (e->vertices()[0]->id() == v_i->id() && e->vertices()[1]->id() == v_j->id()) ||
                (e->vertices()[0]->id() == v_j->id() && e->vertices()[1]->id() == v_i->id()) )
          {
            edge_found = true;
            break;
          }
        }
      }

      // Check if this have been discarted previously
      bool false_cand = stereo_localization::Utils::searchFalseCandidates(false_candidates_, v_i->id(), v_j->id());

      // If no edges found connecting this nodes, try to find loop closures
      if (!false_cand && 
          !edge_found && stereo_localization::Utils::poseDiff(pose_i, pose_j) < candidate_threshold_)
      {
        // Get the data of both nodes from database
        std::string where_i = "(id = " + boost::lexical_cast<std::string>(v_i->id() + 1) + ")";
        std::string where_j = "(id = " + boost::lexical_cast<std::string>(v_j->id() + 1) + ")";
        std::vector< boost::shared_ptr<stereo_localization::GraphNodes> > nodes_i;
        std::vector< boost::shared_ptr<stereo_localization::GraphNodes> > nodes_j;
        pg_db_ptr_->getList(nodes_i, where_i);
        pg_db_ptr_->getList(nodes_j, where_j);
        if (nodes_i.size() == 1 && nodes_j.size() == 1)
        {
          cv::Mat desc_i = cv::Mat_<std::vector<float> >();
          cv::Mat desc_j = cv::Mat_<std::vector<float> >();
          desc_i = stereo_localization::Utils::stdMatrixToCvMat(nodes_i[0]->descriptors_.data());
          desc_j = stereo_localization::Utils::stdMatrixToCvMat(nodes_j[0]->descriptors_.data());

          // Compute matchings
          std::vector<cv::DMatch> matches;
          stereo_localization::Utils::thresholdMatching(desc_i, desc_j, matches, descriptors_threshold_);

          if ((int)matches.size() > matches_threshold_)
          {
            // Extract keypoints and 3d points of node i
            std::vector<cv::Point2f> keypoints_j;
            std::vector<cv::Point3f> points3d_i;
            keypoints_j = stereo_localization::Utils::stdMatrixToCvPoint2f(nodes_j[0]->keypoints_.data());
            points3d_i = stereo_localization::Utils::stdMatrixToCvPoint3f(nodes_i[0]->points3d_.data());
            std::vector<cv::Point2f> matched_keypoints;
            std::vector<cv::Point3f> matched_3d_points;
            for (size_t i = 0; i < matches.size(); ++i)
            {
              int index_left = matches[i].queryIdx;
              int index_right = matches[i].trainIdx;;
              matched_3d_points.push_back(points3d_i[index_left]);
              matched_keypoints.push_back(keypoints_j[index_right]);
            }
            
            // Compute the transformation between the nodes
            cv::Mat rvec, tvec;
            std::vector<int> inliers;
            cv::solvePnPRansac(matched_3d_points, matched_keypoints, camera_matrix_, 
                               cv::Mat(), rvec, tvec, false, 
                               1000, 5.0, 10000, inliers);

            if (static_cast<int>(inliers.size()) >= 10)
            {
              // Good! Loop closure, get the transformation matrix
              tf::Transform cl_edge = stereo_localization::Utils::buildTransformation(rvec, tvec);

              // Add the new edge to graph
              g2o::EdgeSE3* e = new g2o::EdgeSE3();
              Eigen::Isometry3d t = stereo_localization::Utils::tfToEigen(cl_edge);
              e->setVertex(0, v_i);
              e->setVertex(1, v_j);
              e->setMeasurement(t);
              graph_optimizer_.addEdge(e);
              edge_added = true;

              ROS_INFO_STREAM("[GraphConstructor:] Loop closed between nodes " << v_i->id() << " and " << v_j->id());
            }
          }
        }
      }
      else
      {
        // Bad candidate, save to prevent future processes
        if (!false_cand)
        {
          cv::Point2i nodes(v_i->id(), v_j->id());
          false_candidates_.push_back(nodes);
        }
      }
    }
  }

  if (edge_added)
  {
    ROS_INFO("[GraphConstructor:] Optimizing global pose graph...");
    graph_optimizer_.initializeOptimization();
    graph_optimizer_.setVerbose(go2_verbose_);
    graph_optimizer_.optimize(go2_opt_max_iter_);
    ROS_INFO("[GraphConstructor:] Optimization done.");
  }

  // Publish last pose
  if (pose_pub_.getNumSubscribers() > 0 &&
      (int)(graph_optimizer_.vertices().size()-1) > last_pub_pose_ )
  {
    g2o::VertexSE3* last_node =  dynamic_cast<g2o::VertexSE3*>
      (graph_optimizer_.vertices()[graph_optimizer_.vertices().size()-1]);
    tf::Transform last_pose = stereo_localization::Utils::getNodePose(last_node);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = map_frame_id_;
    tf::poseTFToMsg(last_pose, pose_msg.pose);
    pose_pub_.publish(pose_msg);

    last_pub_pose_ = graph_optimizer_.vertices().size()-1;
  }

  block_update_ = false;
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
