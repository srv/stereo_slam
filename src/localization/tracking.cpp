#include <ros/ros.h>

#include "localization/tracking.h"
#include "common/tools.h"

using namespace tools;

namespace slam
{

  Tracking::Tracking(FramePublisher *f_pub, Graph *graph)
                    : f_pub_(f_pub), graph_(graph), reset_fixed_frame_(false)
  {}

  void Tracking::run()
  {
    // Init
    state_ = NOT_INITIALIZED;

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    image_transport::ImageTransport it(nh);
    pose_pub_ = nhp.advertise<nav_msgs::Odometry>("odometry", 1);

    image_transport::SubscriberFilter left_sub, right_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub, right_info_sub;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;

    // Message sync
    boost::shared_ptr<Sync> sync;
    odom_sub      .subscribe(nh, params_.odom_topic, 20);
    left_sub      .subscribe(it, params_.camera_topic+"/left/image_rect", 3);
    right_sub     .subscribe(it, params_.camera_topic+"/right/image_rect", 3);
    left_info_sub .subscribe(nh, params_.camera_topic+"/left/camera_info",  3);
    right_info_sub.subscribe(nh, params_.camera_topic+"/right/camera_info", 3);
    sync.reset(new Sync(SyncPolicy(5), odom_sub, left_sub, right_sub, left_info_sub, right_info_sub) );
    sync->registerCallback(bind(&Tracking::msgsCallback, this, _1, _2, _3, _4, _5));

    ros::spin();
  }

  void Tracking::msgsCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,
                              const sensor_msgs::ImageConstPtr& l_img_msg,
                              const sensor_msgs::ImageConstPtr& r_img_msg,
                              const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                              const sensor_msgs::CameraInfoConstPtr& r_info_msg)
  {

    tf::Transform c_odom_robot = Tools::odomTotf(*odom_msg);

    Mat l_img, r_img;
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
      Tools::getCameraModel(*l_info_msg, *r_info_msg, camera_model_, camera_matrix_);

      // Set graph properties
      graph_->setCamera2Odom(odom2camera_.inverse());
      graph_->setCameraMatrix(camera_matrix_);

      // The initial frame
      f_frame_ = Frame(l_img, r_img, camera_model_);

      // For the first frame, its estimated pose will coincide with odometry
      tf::Transform c_odom_camera = c_odom_robot * odom2camera_;
      f_frame_.setEstimatedPose(c_odom_camera);
      f_frame_.setOdometryPose(c_odom_camera);

      state_ = INITIALIZING;
    }
    else
    {
      // The current frame
      c_frame_ = Frame(l_img, r_img, camera_model_);

      // Set the odometry of this frame
      tf::Transform c_odom_camera = c_odom_robot * odom2camera_;
      c_frame_.setOdometryPose(c_odom_camera);

      // Track this frame
      trackCurrentFrame();
      f_pub_->update(this);
      publishPose();
      needNewFixedFrame();
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

  void Tracking::trackCurrentFrame()
  {

    matches_.clear();
    inliers_.clear();

    // Initial estimation (solvePNP and odometry) of the current frame position
    tf::Transform odom_diff = f_frame_.getOdometryPose().inverse() * c_frame_.getOdometryPose();
    tf::Transform c_pose = f_frame_.getEstimatedPose() * odom_diff;
    c_frame_.setEstimatedPose(c_pose);

    // Descriptor matching
    Mat f_desc = f_frame_.getLeftDesc();
    Mat c_desc = c_frame_.getLeftDesc();
    vector<KeyPoint> f_kp = f_frame_.getLeftKp();
    vector<Point3f> f_points_3d = f_frame_.get3D();
    vector<Point3f> c_points_3d = c_frame_.get3D();

    // Matching
    Mat match_mask;
    const int knn = 2;
    const double ratio = 0.9;
    Ptr<DescriptorMatcher> descriptor_matcher;
    descriptor_matcher = DescriptorMatcher::create("BruteForce-Hamming");
    vector<vector<DMatch> > knn_matches;
    descriptor_matcher->knnMatch(c_desc, f_desc, knn_matches, knn, match_mask);
    for (uint m=0; m<knn_matches.size(); m++)
    {
      if (knn_matches[m].size() < 2) continue;
      if (knn_matches[m][0].distance <= knn_matches[m][1].distance * ratio)
        matches_.push_back(knn_matches[m][0]);
    }

    // Check minimum matches
    if (matches_.size() < MIN_INLIERS)
    {
      c_frame_.setInliers(0);
    }
    else
    {
      vector<Point2f> f_matched_kp;
      vector<Point3f> f_matched_3d_points;
      vector<Point3f> c_matched_3d_points;
      for(int i=0; i<matches_.size(); i++)
      {
        f_matched_kp.push_back(f_kp[matches_[i].trainIdx].pt);
        f_matched_3d_points.push_back(f_points_3d[matches_[i].trainIdx]);
        c_matched_3d_points.push_back(c_points_3d[matches_[i].queryIdx]);
      }

      // Use extrinsic guess when the tracker is not reseting the fixed frame to speed up the process
      bool use_guess = true;
      if (reset_fixed_frame_)
        use_guess = false;

      // Estimate the motion
      solvePnPRansac(c_matched_3d_points, f_matched_kp, camera_matrix_,
                     Mat(), rvec_, tvec_, use_guess,
                     100, 1.1,
                     MAX_INLIERS, inliers_);

      c_frame_.setInliers(inliers_.size());

      if (inliers_.size() > MIN_INLIERS)
      {
        // Estimated pose does not require odometry
        tf::Transform c_pose = f_frame_.getEstimatedPose() * Tools::buildTransformation(rvec_, tvec_);
        c_frame_.setEstimatedPose(c_pose);
      }
    }

    if (pose_pub_.getNumSubscribers() > 0)
    {
      nav_msgs::Odometry pose_msg;
      pose_msg.header.stamp = ros::Time::now();
      tf::poseTFToMsg(c_frame_.getEstimatedPose(), pose_msg.pose.pose);
      pose_pub_.publish(pose_msg);
    }
  }

  void Tracking::needNewFixedFrame()
  {
    // Wait for initialization
    if (state_ == INITIALIZING)
    {
      if (inliers_.size() < MIN_INLIERS)
      {
        f_frame_ = c_frame_;
        return;
      }
      else
        state_ = WORKING;
    }

    // -> System is initialized

    // Reset fixed frame?
    if (inliers_.size() < MIN_INLIERS)
    {
      // Is the system lost?
      if (reset_fixed_frame_ && state_ != LOST)
      {
        // System got lost!
        state_ = LOST;
        lost_time_ = ros::WallTime::now();
        ROS_INFO("Tracker got lost!");
      }

      // Add frames to the graph when not lost
      if (state_ != LOST)
        graph_->addFrameToQueue(f_frame_);

      // New fixed frame needed
      f_frame_ = p_frame_;
      reset_fixed_frame_ = true;
    }
    else
    {
      // Re-localization
      if (state_ == LOST)
      {
        ros::WallDuration lost_duration = ros::WallTime::now() - lost_time_;
        ROS_INFO_STREAM("Tracker found. Lost duration: " << lost_duration.toSec() << " sec.");

        // Set the fixed frame inliers to zero, since its position must be corrected by the graph
        f_frame_.setInliers(0);
      }

      // Do not reset fixed frame
      reset_fixed_frame_ = false;
      state_ = WORKING;
    }

    // Store current frame
    p_frame_ = c_frame_;

  }

  void Tracking::publishPose()
  {
    if (pose_pub_.getNumSubscribers() > 0)
    {
      nav_msgs::Odometry pose_msg;
      pose_msg.header.stamp = ros::Time::now();
      tf::poseTFToMsg(c_frame_.getEstimatedPose(), pose_msg.pose.pose);
      pose_pub_.publish(pose_msg);
    }
  }

} //namespace slam