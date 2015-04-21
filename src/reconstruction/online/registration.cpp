#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <boost/thread.hpp>
#include "common/tools.h"
#include "stereo_slam/SlamVertex.h"
#include "stereo_slam/SlamEdge.h"
#include "stereo_slam/Correction.h"

using namespace std;
using namespace tools;
using namespace boost;

typedef pcl::PointXYZRGB                                PointRGB;
typedef pcl::PointCloud<PointRGB>                       PointCloudRGB;
typedef pcl::IterativeClosestPoint<PointRGB, PointRGB>  IterativeClosestPoint;

/** \brief Catches the Ctrl+C signal.
  */
void stopHandler(int s)
{
  printf("Caught signal %d\n", s);
  exit(1);
}

class Registration
{

private:

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Topic subscribers
  ros::Subscriber edge_sub_;
  message_filters::Subscriber<stereo_slam::SlamVertex> vertex_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;

  // Topic publishers
  ros::Publisher correction_pub_;

  // Topic sync properties (with pointcloud)
  typedef message_filters::sync_policies::ApproximateTime<stereo_slam::SlamVertex,
                                                          sensor_msgs::PointCloud2> PolicyCloud;
  typedef message_filters::Synchronizer<PolicyCloud> SyncCloud;
  boost::shared_ptr<SyncCloud> sync_cloud_;

  // Node parameters
  bool lock_;
  string work_dir_;
  string cloud_topic_;
  string vertex_topic_;
  string edge_topic_;
  int cloud_queue_;
  int last_node_id_;
  double voxel_size_;
  vector< pair<int, tf::Transform> > cloud_poses_;
  vector< int > consecutive_aligned_;

public:

  /**
   * Class constructor
   */
  Registration() : nh_(), nh_private_("~")
  {
    // Setup the signal handler
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = stopHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    // Read parameters
    readParameters();

    // Init class
    init();
  }


  /** \brief Edge callback
  * @return
  * \param edge_msg message of type stereo_slam::SlamEdge
  */
  void edgeCallback(const stereo_slam::SlamEdge::ConstPtr& edge_msg)
  {
    int node_0 = edge_msg->node_0;
    int node_1 = edge_msg->node_1;

    // Check if clouds have been saved by the vertexCallback
    string cloud_0_filename = work_dir_ + lexical_cast<string>(node_0) + ".pcd";
    string cloud_1_filename = work_dir_ + lexical_cast<string>(node_1) + ".pcd";
    PointCloudRGB::Ptr cloud_0(new PointCloudRGB);
    PointCloudRGB::Ptr cloud_1(new PointCloudRGB);
    if (pcl::io::loadPCDFile<PointRGB> (cloud_0_filename, *cloud_0) == -1)
    {
      ROS_WARN_STREAM("[Registration:] Couldn't read the file: " << cloud_0_filename);
      return;
    }
    if (pcl::io::loadPCDFile<PointRGB> (cloud_1_filename, *cloud_1) == -1)
    {
      ROS_WARN_STREAM("[Registration:] Couldn't read the file: " << cloud_1_filename);
      return;
    }

    // Extract the transform between clouds
    tf::Vector3 t(edge_msg->x, edge_msg->y, edge_msg->z);
    tf::Quaternion q(edge_msg->qx, edge_msg->qy, edge_msg->qz, edge_msg->qw);
    tf::Transform tf(q, t);

    // Transform cloud
    Eigen::Affine3d tf_eigen;
    transformTFToEigen(tf, tf_eigen);
    pcl::transformPointCloud(*cloud_1, *cloud_1, tf_eigen);

    // Align
    bool converged;
    double score;
    tf::Transform correction;
    if ( !pairAlign(cloud_1, cloud_0, correction, converged, score) )
    {
      ROS_WARN_STREAM("[Registration:] Pair align between " << node_0 <<
        " and " << node_1 << " discarted (convergence: " <<
          converged << ", score: " << score << ").");
      return;
    }

    double distance = sqrt(correction.getOrigin().x()*correction.getOrigin().x() +
                           correction.getOrigin().y()*correction.getOrigin().y() +
                           correction.getOrigin().z()*correction.getOrigin().z());
    ROS_INFO_STREAM("[Registration:] Correction between " << node_0 << " and " <<
      node_1 << " is: " <<
      r4d(correction.getOrigin().x()) << ", " <<
      r4d(correction.getOrigin().y()) << ", " <<
      r4d(correction.getOrigin().z()) << " (dist -> " << distance << ", score: " << score << ").");

    // Distance threshold
    double max_correction = 0.25;
    if (distance > max_correction) {
      ROS_WARN_STREAM("[Registration:] Pointcloud discarted due to its large correction (>" << max_correction << ").");
      return;
    }

    // Build the final transform
    tf::Transform final_tf = tf * correction;

    // Publish
    if (correction_pub_.getNumSubscribers() > 0)
    {
      stereo_slam::Correction correction_msg;
      correction_msg.header.stamp = ros::Time::now();
      correction_msg.node_0 = node_0;
      correction_msg.node_1 = node_1;
      correction_msg.x = final_tf.getOrigin().x();
      correction_msg.y = final_tf.getOrigin().y();
      correction_msg.z = final_tf.getOrigin().z();
      correction_msg.qx = final_tf.getRotation().x();
      correction_msg.qy = final_tf.getRotation().y();
      correction_msg.qz = final_tf.getRotation().z();
      correction_msg.qw = final_tf.getRotation().w();
      correction_pub_.publish(correction_msg);
    }
  }


  /** \brief Vertex callback. This function is called when synchronized vertex and cloud
  * message are received.
  * @return
  * \param vertex_msg message of type stereo_slam::SlamVertex
  * \param cloud_msg ros pointcloud message of type sensor_msgs::PointCloud2
  */
  void vertexCallback(const stereo_slam::SlamVertex::ConstPtr& vertex_msg,
                      const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    // Check
    int node_id = vertex_msg->node_id;
    if (node_id - last_node_id_ > 1)
      ROS_WARN_STREAM("[Registration:] We are missing clouds, please increase the cloud_queue parameter (now: " << cloud_queue_ << ").");
    last_node_id_ = node_id;

    // Get the cloud and filter
    PointCloudRGB::Ptr cloud(new PointCloudRGB);
    fromROSMsg(*cloud_msg, *cloud);
    cloud = filter(cloud, voxel_size_);

    // Lock
    while(lock_) {}
      lock_ = true;

    // Save the cloud
    pcl::io::savePCDFileBinary(work_dir_ + lexical_cast<string>(node_id) + ".pcd", *cloud);

    // Extract the pose
    tf::Vector3 t(vertex_msg->x, vertex_msg->y, vertex_msg->z);
    tf::Quaternion q(vertex_msg->qx, vertex_msg->qy, vertex_msg->qz, vertex_msg->qw);
    tf::Transform pose(q, t);
    cloud_poses_.push_back(make_pair(node_id, pose));

    // Unlock
    lock_ = false;

    // Log
    ROS_INFO_STREAM("[Registration:] Received cloud " << node_id << ".");
  }

  /** \brief Thread to align the pointclouds
   */
  void alignThread()
  {
    // Frequency
    ros::WallDuration d(0.01);

    while(true)
    {
      // Wait
      d.sleep();

      // Lock
      while(lock_) {}
        lock_ = true;

      // Determine the next cloud to be aligned
      int to_be_aligned = -1;
      for (uint i=1; i<cloud_poses_.size(); i++)
      {
        // Get the node and check if it has been aligned
        bool not_aligned = true;
        int node_id = cloud_poses_[i].first;

        for (uint j=0; j<consecutive_aligned_.size(); j++)
        {
          if (consecutive_aligned_[j] == node_id)
          {
            not_aligned = false;
            break;
          }
        }

        if (not_aligned)
        {
          to_be_aligned = node_id;
          break;
        }
      }

      // Unlock
      lock_ = false;

      // Any cloud to be aligned?
      if (to_be_aligned > 0)
      {
        // Mark as aligned
        consecutive_aligned_.push_back(to_be_aligned);

        // Get the clouds
        uint idx_0 = -1;
        uint idx_1 = -1;
        for (uint i=0; i<cloud_poses_.size(); i++)
        {
          if (cloud_poses_[i].first == to_be_aligned-1)
            idx_0 = i;
          if (cloud_poses_[i].first == to_be_aligned)
            idx_1 = i;
          if (idx_0 > -1 && idx_1 > -1)
            break;
        }

        // Sanity check
        if (idx_0 < 0 || idx_1 < 0) continue;

        string cloud_0_file = work_dir_ + lexical_cast<string>(cloud_poses_[idx_0].first) + ".pcd";
        string cloud_1_file = work_dir_ + lexical_cast<string>(cloud_poses_[idx_1].first) + ".pcd";
        PointCloudRGB::Ptr cloud_0(new PointCloudRGB);
        PointCloudRGB::Ptr cloud_1(new PointCloudRGB);
        if (pcl::io::loadPCDFile<PointRGB>(cloud_0_file, *cloud_0) == -1)
        {
          ROS_ERROR_STREAM("[Registration:] Couldn't read the file: " << cloud_0_file);
          continue;
        }
        if (pcl::io::loadPCDFile<PointRGB>(cloud_1_file, *cloud_1) == -1)
        {
          ROS_ERROR_STREAM("[Registration:] Couldn't read the file: " << cloud_1_file);
          continue;
        }

        // Transform current cloud according to slam pose
        tf::Transform tf_0 = cloud_poses_[idx_0].second;
        tf::Transform tf_1 = cloud_poses_[idx_1].second;

        tf::Transform tf_01 = tf_0.inverse() * tf_1;
        Eigen::Affine3d tf_01_eigen;
        transformTFToEigen(tf_01, tf_01_eigen);
        pcl::transformPointCloud(*cloud_1, *cloud_1, tf_01_eigen);

        // TODO: check overlap!!!!!

        // Align
        bool converged;
        double score;
        tf::Transform correction;
        if ( !pairAlign(cloud_1, cloud_0, correction, converged, score) )
        {
          ROS_WARN_STREAM("[Registration:] Pair align between " << to_be_aligned-1 <<
            " and " << to_be_aligned << " discarted (convergence: " <<
              converged << ", score: " << score << ").");
          continue;
        }

        double distance = sqrt(correction.getOrigin().x()*correction.getOrigin().x() +
                               correction.getOrigin().y()*correction.getOrigin().y() +
                               correction.getOrigin().z()*correction.getOrigin().z());
        ROS_INFO_STREAM("[Registration:] Correction between " << to_be_aligned-1 << " and " <<
          to_be_aligned << " is: " <<
          r4d(correction.getOrigin().x()) << ", " <<
          r4d(correction.getOrigin().y()) << ", " <<
          r4d(correction.getOrigin().z()) << " (dist -> " << distance << ", score: " << score << ").");

        // Distance threshold
        double max_correction = 0.2;
        if (distance > max_correction) {
          ROS_WARN_STREAM("[Registration:] Pointcloud discarted due to its large correction (>" << max_correction << ").");
          continue;
        }

        // The final position of this node is
        tf::Transform final_tf = tf_01 * correction;

        // Publish
        if (correction_pub_.getNumSubscribers() > 0)
        {
          stereo_slam::Correction correction_msg;
          correction_msg.header.stamp = ros::Time::now();
          correction_msg.node_0 = to_be_aligned-1;
          correction_msg.node_1 = to_be_aligned;
          correction_msg.x = final_tf.getOrigin().x();
          correction_msg.y = final_tf.getOrigin().y();
          correction_msg.z = final_tf.getOrigin().z();
          correction_msg.qx = final_tf.getRotation().x();
          correction_msg.qy = final_tf.getRotation().y();
          correction_msg.qz = final_tf.getRotation().z();
          correction_msg.qw = final_tf.getRotation().w();
          correction_pub_.publish(correction_msg);
        }
      }
    }
  }


  /** \brief Align 2 pointclouds and return the transformation
    * @return true if alignment successful.
    * \param source cloud
    * \param target cloud
    * \param the output transformation
    */
  bool pairAlign(PointCloudRGB::Ptr src,
                 PointCloudRGB::Ptr tgt,
                 tf::Transform &output,
                 bool &converged,
                 double &score)
  {
    PointCloudRGB::Ptr aligned (new PointCloudRGB);
    IterativeClosestPoint icp;
    icp.setMaxCorrespondenceDistance(0.07);
    icp.setRANSACOutlierRejectionThreshold(0.005);
    icp.setTransformationEpsilon(0.000001);
    icp.setEuclideanFitnessEpsilon(0.0001);
    icp.setMaximumIterations(100);
    icp.setInputSource(src);
    icp.setInputTarget(tgt);
    icp.align(*aligned);

    // The transform
    output = Tools::matrix4fToTf(icp.getFinalTransformation());

    // The measures
    converged = icp.hasConverged();
    score = icp.getFitnessScore();

    // Return valid or not
    return ( converged && (score < 1.0) );
  }


  /** \brief Filter a pointcloud.
   * @return the cloud filtered.
   * \param cloud to be filtered.
   * \param voxel size for the voxel grid filter.
   */
  PointCloudRGB::Ptr filter(PointCloudRGB::Ptr in_cloud, double voxel_size)
  {
    // Remove nans
    vector<int> indices;
    PointCloudRGB::Ptr cloud(new PointCloudRGB);
    pcl::removeNaNFromPointCloud(*in_cloud, *cloud, indices);
    indices.clear();

    // Voxel grid filter
    pcl::ApproximateVoxelGrid<PointRGB> grid;
    grid.setLeafSize(voxel_size, voxel_size, 0.5);
    grid.setDownsampleAllData(true);
    grid.setInputCloud(cloud);
    grid.filter(*cloud);

    // Remove isolated points
    pcl::RadiusOutlierRemoval<PointRGB> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.04);
    outrem.setMinNeighborsInRadius(50);
    outrem.filter(*cloud);
    pcl::StatisticalOutlierRemoval<PointRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(40);
    sor.setStddevMulThresh(2.0);
    sor.filter(*cloud);

    return cloud;
  }


  /** \brief Round 4 decimals
   * @return the rounded value
   * \param value to be rounded
   */
  float r4d(float f)
  {
    return roundf(f * 10000) / 10000;
  }


  /** \brief Reads the node parameters
   */
  void readParameters()
  {
    nh_private_.param("work_dir",     work_dir_,      string(""));
    nh_private_.param("cloud_topic",  cloud_topic_,   string(""));
    nh_private_.param("vertex_topic", vertex_topic_,  string(""));
    nh_private_.param("edge_topic",   edge_topic_,  string(""));
    nh_private_.param("cloud_queue",  cloud_queue_,   20);
    nh_private_.param("voxel_size",   voxel_size_,    0.005);


    // Init workspace
    if (work_dir_[work_dir_.length()-1] != '/')
      work_dir_ += "/";
  }


  /** \brief Initialize class
   */
  void init()
  {
    // Initialize
    lock_ = false;
    last_node_id_ = 0; // Should be -1, but the system usually lose the first cloud :(

    // Correction publisher
    correction_pub_ = nh_private_.advertise<stereo_slam::Correction>("correction", 3);

    // Create the edge callback
    edge_sub_ = nh_.subscribe<stereo_slam::SlamEdge>(edge_topic_, 5, &Registration::edgeCallback, this);

    // Create the vertex callback
    vertex_sub_.subscribe(nh_, vertex_topic_, 20);
    cloud_sub_.subscribe(nh_, cloud_topic_, cloud_queue_);
    sync_cloud_.reset(new SyncCloud(PolicyCloud(cloud_queue_),
                                    vertex_sub_,
                                    cloud_sub_) );
    sync_cloud_->registerCallback(bind(
        &Registration::vertexCallback,
        this, _1, _2));
  }

};


int main(int argc, char** argv)
{
  // Start the node
  ros::init(argc, argv, "registration");
  Registration node;

  // Start the align thread
  boost::thread align_thread( boost::bind(&Registration::alignThread, &node) );

  // ROS spin
  ros::MultiThreadedSpinner spinner(0);
  spinner.spin();

  // Join, delete and exit
  align_thread.join();
  return 0;
}

