#include <ros/ros.h>

#include "tracking.h"
#include "tools.h"

using namespace tools;

namespace slam
{

  Tracking::Tracking(Publisher *f_pub, Graph *graph)
                    : f_pub_(f_pub), graph_(graph), frame_id_(0)
  {}

  void Tracking::run()
  {
    // Init
    state_ = NOT_INITIALIZED;
    last_fixed_frame_pose_.setIdentity();

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    image_transport::ImageTransport it(nh);

    image_transport::SubscriberFilter left_sub, right_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub, right_info_sub;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub;

    // Message sync
    boost::shared_ptr<Sync> sync;
    odom_sub      .subscribe(nh, params_.odom_topic, 20);
    left_sub      .subscribe(it, params_.camera_topic+"/left/image_rect_color", 3);
    right_sub     .subscribe(it, params_.camera_topic+"/right/image_rect_color", 3);
    left_info_sub .subscribe(nh, params_.camera_topic+"/left/camera_info",  3);
    right_info_sub.subscribe(nh, params_.camera_topic+"/right/camera_info", 3);
    cloud_sub     .subscribe(nh, params_.camera_topic+"/points2", 5);
    sync.reset(new Sync(SyncPolicy(5), odom_sub, left_sub, right_sub, left_info_sub, right_info_sub, cloud_sub) );
    sync->registerCallback(bind(&Tracking::msgsCallback, this, _1, _2, _3, _4, _5, _6));

    // Create directory to store the keyframes
    string keyframes_dir = WORKING_DIRECTORY + "keyframes";
    if (fs::is_directory(keyframes_dir))
      fs::remove_all(keyframes_dir);
    fs::path dir1(keyframes_dir);
    if (!fs::create_directory(dir1))
      ROS_ERROR("[Localization:] ERROR -> Impossible to create the keyframes directory.");

    // Create directory to store the pointclouds
    string pointclouds_dir = WORKING_DIRECTORY + "pointclouds";
    if (fs::is_directory(pointclouds_dir))
      fs::remove_all(pointclouds_dir);
    fs::path dir2(pointclouds_dir);
    if (!fs::create_directory(dir2))
      ROS_ERROR("[Localization:] ERROR -> Impossible to create the pointclouds directory.");

    ros::spin();
  }

  void Tracking::msgsCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,
                              const sensor_msgs::ImageConstPtr& l_img_msg,
                              const sensor_msgs::ImageConstPtr& r_img_msg,
                              const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                              const sensor_msgs::CameraInfoConstPtr& r_info_msg,
                              const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {

    tf::Transform c_odom_robot = Tools::odomTotf(*odom_msg);
    double timestamp = l_img_msg->header.stamp.toSec();

    cv::Mat l_img, r_img;
    Tools::imgMsgToMat(*l_img_msg, *r_img_msg, l_img, r_img);

    if (state_ == NOT_INITIALIZED)
    {
      // Transformation between odometry and camera
      if (!getOdom2CameraTf(*odom_msg, *l_img_msg, odom2camera_))
      {
        ROS_WARN("[Localization:] Impossible to transform odometry to camera frame.");
        return;
      }

      // Camera parameters
      cv::Mat camera_matrix;
      Tools::getCameraModel(*l_info_msg, *r_info_msg, camera_model_, camera_matrix);

      ROS_INFO_STREAM("TF Camera to odometry frame:");
      ROS_INFO_STREAM("T: " << odom2camera_.getOrigin().x() << ", " << odom2camera_.getOrigin().y() << ", " << odom2camera_.getOrigin().z());
      ROS_INFO_STREAM("Q: " << odom2camera_.getRotation().x() << ", " << odom2camera_.getRotation().y() << ", " << odom2camera_.getRotation().z() << ", " << odom2camera_.getRotation().w());

      // Set graph properties
      graph_->setCamera2Odom(odom2camera_.inverse());
      graph_->setCameraMatrix(camera_matrix);
      graph_->setCameraModel(camera_model_.left());

      // The initial frame
      c_frame_ = Frame(l_img, r_img, camera_model_, timestamp);

      // For the first frame, its estimated pose will coincide with odometry
      tf::Transform c_odom_camera = c_odom_robot * odom2camera_;
      c_frame_.setCameraPose(c_odom_camera);

      state_ = INITIALIZING;
    }
    else
    {
      PointCloudRGB::Ptr pcl_cloud(new PointCloudRGB);
      fromROSMsg(*cloud_msg, *pcl_cloud);

      // The current frame
      c_frame_ = Frame(l_img, r_img, camera_model_, timestamp);

      // Set the odometry of this frame
      tf::Transform c_odom_camera = c_odom_robot * odom2camera_;
      c_frame_.setCameraPose(c_odom_camera);

      // Need new keyframe
      needNewKeyFrame(pcl_cloud);
    }
  }

  bool Tracking::getOdom2CameraTf(nav_msgs::Odometry odom_msg,
                                  sensor_msgs::Image img_msg,
                                  tf::StampedTransform &transform)
  {
    // Init the transform
    transform.setIdentity();

    try
    {
      // Extract the transform
      tf_listener_.lookupTransform(odom_msg.child_frame_id,
                                   img_msg.header.frame_id,
                                   ros::Time(0),
                                   transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("%s", ex.what());
      return false;
    }
    return true;
  }

  void Tracking::needNewKeyFrame(PointCloudRGB::Ptr cloud)
  {
    // Init initialization
    if (state_ == INITIALIZING)
    {
      addFrameToMap(c_frame_, cloud);
      state_ = WORKING;
    }
    else
    {
      // Do not add very close frames
      double pose_diff = Tools::poseDiff(last_fixed_frame_pose_, c_frame_.getCameraPose());
      if (pose_diff > 0.6)
      {
        addFrameToMap(c_frame_, cloud);
      }
    }
  }

  void Tracking::addFrameToMap(Frame frame, PointCloudRGB::Ptr cloud)
  {
    if (frame.getLeftKp().size() > LC_MIN_INLIERS)
    {
      frame.regionClustering();

      // Check if clusters
      vector< vector<int> > clusters = frame.getClusters();
      if (clusters.size() > 0)
      {
        // Add to graph
        f_pub_->publishClustering(frame);
        graph_->addFrameToQueue(frame);
        last_fixed_frame_pose_ = frame.getCameraPose();

        // Save cloud
        PointCloudRGB::Ptr cloud_filtered;
        cloud_filtered = filterCloud(cloud);
        if (cloud_filtered->points.size() == 0) return;
        string pc_filename = WORKING_DIRECTORY + "pointclouds/" + lexical_cast<string>(frame_id_) + ".pcd";
        pcl::io::savePCDFileBinary(pc_filename, *cloud_filtered);
        frame_id_++;
      }
    }
  }

  PointCloudRGB::Ptr Tracking::filterCloud(PointCloudRGB::Ptr in_cloud)
  {
    // Remove nans
    vector<int> indicies;
    PointCloudRGB::Ptr cloud(new PointCloudRGB);
    pcl::removeNaNFromPointCloud(*in_cloud, *cloud, indicies);

    // Voxel grid filter (used as x-y surface extraction. Note that leaf in z is very big)
    pcl::ApproximateVoxelGrid<PointRGB> grid;
    grid.setLeafSize(0.005, 0.005, 0.01);
    grid.setDownsampleAllData(true);
    grid.setInputCloud(cloud);
    grid.filter(*cloud);

    return cloud;
  }

} //namespace slam