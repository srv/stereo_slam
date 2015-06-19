#include <ros/ros.h>

#include "localization/tracking.h"
#include "common/tools.h"

using namespace tools;

namespace slam
{

  Tracking::Tracking(FramePublisher *f_pub) : f_pub_(f_pub), reset_fixed_frame_(false)
  {}

  void Tracking::run()
  {
    // Init
    state_ = NOT_INITIALIZED;

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

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

      // The initial frame
      f_frame_ = Frame(l_img, r_img, camera_model_);

      state_ = WORKING;
    }
    else
    {
      // The current frame
      c_frame_ = Frame(l_img, r_img, camera_model_);

      trackCurrentFrame();
      f_pub_->update(this);
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

    // Descriptor matching
    Mat p_desc = f_frame_.getLeftDesc();
    Mat c_desc = c_frame_.getLeftDesc();
    vector<KeyPoint> c_kp = c_frame_.getLeftKp();
    vector<Point3f> p_points_3d = f_frame_.get3D();

    // Matching
    Mat match_mask;
    const int knn = 2;
    const double ratio = 0.8;
    Ptr<DescriptorMatcher> descriptor_matcher;
    descriptor_matcher = DescriptorMatcher::create("BruteForce");
    vector<vector<DMatch> > knn_matches;
    descriptor_matcher->knnMatch(p_desc, c_desc, knn_matches, knn, match_mask);
    for (uint m=0; m<knn_matches.size(); m++)
    {
      if (knn_matches[m].size() < 2) continue;
      if (knn_matches[m][0].distance <= knn_matches[m][1].distance * ratio)
        matches_.push_back(knn_matches[m][0]);
    }

    // Check minimum
    if (matches_.size() < 50)
      return;

    vector<Point2f> c_matched_kp;
    vector<Point3f> p_matched_3d_points;
    for(int i=0; i<matches_.size(); i++)
    {
      c_matched_kp.push_back(c_kp[matches_[i].trainIdx].pt);
      p_matched_3d_points.push_back(p_points_3d[matches_[i].queryIdx]);
    }

    // Use extrinsic guess when the tracker is not reseting the fixed frame to speed up the process
    bool use_guess = true;
    if (reset_fixed_frame_)
      use_guess = false;

    // Estimate the motion
    solvePnPRansac(p_matched_3d_points, c_matched_kp, camera_matrix_,
                   cv::Mat(), rvec_, tvec_, use_guess,
                   100, 3.0,
                   30, inliers_);
  }

  void Tracking::needNewFixedFrame()
  {
    if (inliers_.size() < 30)
    {
      if (reset_fixed_frame_)
      {
        // System got lost!
        ROS_INFO("Tracker got lost!");
      }

      // New fixed frame needed
      f_frame_ = p_frame_;
      p_frame_ = c_frame_;
      reset_fixed_frame_ = true;
    }
    else
    {
      reset_fixed_frame_ = false;
      p_frame_ = c_frame_;
    }

  }

} //namespace slam