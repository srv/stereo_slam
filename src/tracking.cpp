#include <ros/ros.h>

#include "tracking.h"
#include "tools.h"

using namespace tools;

namespace slam
{

  Tracking::Tracking(Publisher *f_pub, Graph *graph)
    : f_pub_(f_pub), graph_(graph), frame_id_(0), jump_detected_(false), secs_to_filter_(10.0)
  {}

  void Tracking::run()
  {
    // Init
    state_ = NOT_INITIALIZED;
    last_fixed_frame_pose_.setIdentity();

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    pose_pub_ = nhp.advertise<nav_msgs::Odometry>("odometry", 1);
    pc_pub_ = nhp.advertise<sensor_msgs::PointCloud2>("pointcloud", 5);
    overlapping_pub_ = nhp.advertise<sensor_msgs::Image>("tracking_overlap", 1, true);

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

      // Set graph properties
      graph_->setCamera2Odom(odom2camera_.inverse());
      graph_->setCameraMatrix(camera_matrix);
      graph_->setCameraModel(camera_model_.left());

      // The initial frame
      c_frame_ = Frame(l_img, r_img, camera_model_, timestamp);

      // For the first frame, its estimated pose will coincide with odometry
      tf::Transform c_odom_camera = c_odom_robot * odom2camera_;
      c_frame_.setCameraPose(c_odom_camera);

      prev_corrected_odom_robot_ = c_odom_robot;

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

    // Correct robot pose with the last graph information
    tf::Transform c_odom_camera = c_odom_robot * odom2camera_;
    tf::Transform corrected_odom_robot = graph_->correctOdometry(c_odom_camera) * odom2camera_.inverse();

    // Detect a big jump
    double jump = Tools::poseDiff3D(corrected_odom_robot, prev_corrected_odom_robot_);
    if (!jump_detected_ && jump > 0.8)
    {
      jump_time_ = ros::WallTime::now();
      jump_detected_ = true;
    }
    if (jump_detected_ && ( ros::WallTime::now().toSec() - jump_time_.toSec() > secs_to_filter_ )    )
    {
      jump_detected_ = false;
    }

    tf::Transform pose = corrected_odom_robot;
    if (jump_detected_)
    {
      // Filter big jumps
      double m = 1/(10*secs_to_filter_);
      double factor = m * (ros::WallTime::now().toSec() - jump_time_.toSec());

      double c_x = pose.getOrigin().x();
      double c_y = pose.getOrigin().y();
      double c_z = pose.getOrigin().z();

      double p_x = prev_corrected_odom_robot_.getOrigin().x();
      double p_y = prev_corrected_odom_robot_.getOrigin().y();
      double p_z = prev_corrected_odom_robot_.getOrigin().z();

      double x = factor * c_x + (1-factor) * p_x;
      double y = factor * c_y + (1-factor) * p_y;
      double z = factor * c_z + (1-factor) * p_z;

      tf::Vector3 filtered_pose(x, y, z);
      pose.setOrigin(filtered_pose);
    }

    // Publish
    nav_msgs::Odometry pose_msg = *odom_msg;
    tf::poseTFToMsg(pose, pose_msg.pose.pose);
    pose_pub_.publish(pose_msg);

    // Store
    prev_corrected_odom_robot_ = pose;
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
    // Filter cloud
    PointCloudRGB::Ptr cloud_filtered;
    cloud_filtered = filterCloud(cloud);

    // Init initialization
    if (state_ == INITIALIZING)
    {
      addFrameToMap(c_frame_, cloud_filtered);
      state_ = WORKING;
    }
    else
    {
      // Compute overlap to decide if new keyframe is needed.

      // The transformation between last and current keyframe
      tf::Transform last_2_current = last_fixed_frame_pose_.inverse() * c_frame_.getCameraPose();

      // Speedup the process by converting the current pointcloud to xyz
      PointCloudXYZ::Ptr cloud_xyz(new PointCloudXYZ);
      pcl::copyPointCloud(*cloud_filtered, *cloud_xyz);

      // Transform the current pointcloud
      Eigen::Affine3d tf_eigen;
      transformTFToEigen(last_2_current, tf_eigen);
      PointCloudXYZ::Ptr cloud_xyz_moved(new PointCloudXYZ);
      pcl::transformPointCloud(*cloud_xyz, *cloud_xyz_moved, tf_eigen);

      // Remove the points that are outside the current pointcloud
      PointCloudXYZ::Ptr output(new PointCloudXYZ);
      pcl::CropBox<PointXYZ> crop_filter;
      crop_filter.setInputCloud(cloud_xyz_moved);
      crop_filter.setMin(last_min_pt_);
      crop_filter.setMax(last_max_pt_);
      crop_filter.filter(*output);

      // The overlap estimation
      float overlap = output->points.size() * 100 / cloud_xyz_moved->points.size();

      // Safety factor
      overlap = (0.002 * overlap + 0.8) * overlap;

      // Publish debugging image
      if (overlapping_pub_.getNumSubscribers() > 0)
        publishOverlap(cloud_xyz, last_2_current, overlap);

      // Add frame when overlap is less than...
      if (overlap < TRACKING_MIN_OVERLAP)
        addFrameToMap(c_frame_, cloud_filtered);
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

        // Store minimum and maximum values of last pointcloud
        pcl::getMinMax3D(*cloud, last_min_pt_, last_max_pt_);

        // Publish cloud
        if (pc_pub_.getNumSubscribers() > 0)
        {
          if (cloud->points.size() == 0) return;

          // Publish
          string frame_id_str = Tools::convertTo5digits(frame_id_);
          sensor_msgs::PointCloud2 cloud_msg;
          pcl::toROSMsg(*cloud, cloud_msg);
          cloud_msg.header.frame_id = frame_id_str; // write the keyframe id to the frame id of the message ;)
          pc_pub_.publish(cloud_msg);
        }

        // Increase the frame id counter
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
    grid.setLeafSize(0.08, 0.08, 0.1);
    grid.setDownsampleAllData(true);
    grid.setInputCloud(cloud);
    grid.filter(*cloud);

    return cloud;
  }

  void Tracking::publishOverlap(PointCloudXYZ::Ptr cloud, tf::Transform movement, float overlap)
  {
    int w = 512;
    int h = 384;
    cv::Mat img(h, w, CV_8UC3, cv::Scalar(0,0,0));

    // Draw last fixed frame bounding box
    cv::rectangle(img, cv::Point(3*w/8, 3*h/8), cv::Point(5*w/8, 5*h/8), cv::Scalar(255, 255, 255) );

    float w_scale = (w/4) / (last_max_pt_(0) - last_min_pt_(0));
    float h_scale = (h/4) / (last_max_pt_(1) - last_min_pt_(1));

    // Get boundaries and transform them
    PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    tf::Vector3 p1(min_pt.x, max_pt.y, 0.0);
    tf::Vector3 p2(max_pt.x, max_pt.y, 0.0);
    tf::Vector3 p3(max_pt.x, min_pt.y, 0.0);
    tf::Vector3 p4(min_pt.x, min_pt.y, 0.0);
    p1 = movement * p1;
    p2 = movement * p2;
    p3 = movement * p3;
    p4 = movement * p4;

    // Draw polygon
    cv::line(img, cv::Point(w/2 + p1.x()*w_scale, h/2 + p1.y()*h_scale), cv::Point(w/2 + p2.x()*w_scale, h/2 + p2.y()*h_scale), cv::Scalar(255,0,0));
    cv::line(img, cv::Point(w/2 + p2.x()*w_scale, h/2 + p2.y()*h_scale), cv::Point(w/2 + p3.x()*w_scale, h/2 + p3.y()*h_scale), cv::Scalar(255,0,0));
    cv::line(img, cv::Point(w/2 + p3.x()*w_scale, h/2 + p3.y()*h_scale), cv::Point(w/2 + p4.x()*w_scale, h/2 + p4.y()*h_scale), cv::Scalar(255,0,0));
    cv::line(img, cv::Point(w/2 + p4.x()*w_scale, h/2 + p4.y()*h_scale), cv::Point(w/2 + p1.x()*w_scale, h/2 + p1.y()*h_scale), cv::Scalar(255,0,0));

    // Insert text
    stringstream s;
    s << "Overlap: " << (int)overlap << "%";
    cv::putText(img, s.str(), cv::Point(30, h-20), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255), 2, 8);

    // Publish
    cv_bridge::CvImage ros_image;
    ros_image.image = img.clone();
    ros_image.header.stamp = ros::Time::now();
    ros_image.encoding = "bgr8";
    overlapping_pub_.publish(ros_image.toImageMsg());
  }

} //namespace slam
