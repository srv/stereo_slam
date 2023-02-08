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

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    pose_pub_ = nhp.advertise<nav_msgs::Odometry>("odometry", 1);
    overlapping_pub_ = nhp.advertise<sensor_msgs::Image>("tracking_overlap", 1, true);

    // Create directory to store the keyframes
    string keyframes_dir = params_.working_directory + "keyframes";
    if (fs::is_directory(keyframes_dir))
      fs::remove_all(keyframes_dir);
    fs::path dir1(keyframes_dir);
    if (!fs::create_directory(dir1))
      ROS_ERROR("[Localization:] ERROR -> Impossible to create the keyframes directory.");

    // Create directory to store the clusters
    string clusters_dir = params_.working_directory + "clusters";
    if (fs::is_directory(clusters_dir))
      fs::remove_all(clusters_dir);
    fs::path dir2(clusters_dir);
    if (!fs::create_directory(dir2))
      ROS_ERROR("[Localization:] ERROR -> Impossible to create the clusters directory.");

    // Subscribers
    image_transport::ImageTransport it(nh);
    image_transport::SubscriberFilter left_sub, right_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub, right_info_sub;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;

    // Message sync
    boost::shared_ptr<Sync> sync;
    odom_sub      .subscribe(nh, "odom", 20);
    left_sub      .subscribe(it, "left_image_rect_color", 5);
    right_sub     .subscribe(it, "right_image_rect_color", 5);
    left_info_sub .subscribe(nh, "left_camera_info",  5);
    right_info_sub.subscribe(nh, "right_camera_info", 5);
    sync.reset(new Sync(SyncPolicy(10), odom_sub, left_sub, right_sub, left_info_sub, right_info_sub) );
    sync->registerCallback(bind(&Tracking::msgsCallback, this, _1, _2, _3, _4, _5));


    ros::spin();
  }

  void Tracking::msgsCallback(
    const nav_msgs::Odometry::ConstPtr& odom_msg,
    const sensor_msgs::ImageConstPtr& l_img_msg,
    const sensor_msgs::ImageConstPtr& r_img_msg,
    const sensor_msgs::CameraInfoConstPtr& l_info_msg,
    const sensor_msgs::CameraInfoConstPtr& r_info_msg)
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
      Tools::getCameraModel(*l_info_msg, *r_info_msg, camera_model_, camera_matrix_);

      // Set graph properties
      graph_->setCamera2Odom(odom2camera_.inverse());
      graph_->setCameraMatrix(camera_matrix_);
      graph_->setCameraModel(camera_model_.left());

      // The initial frame
      c_frame_ = Frame(l_img, r_img, camera_model_, timestamp);

      // No corrections apply yet
      prev_robot_pose_ = c_odom_robot;

      // For the first frame, its estimated pose will coincide with odometry
      tf::Transform c_odom_camera = c_odom_robot * odom2camera_;
      c_frame_.setCameraPose(c_odom_camera);

      // Publish stereo matches
      f_pub_->publishStereoMatches(c_frame_);

      bool frame_ok = addFrameToMap();
      if (frame_ok)
      {
        // Store the odometry for this frame
        odom_pose_history_.push_back(c_odom_camera);

        // Mark as initialized
        state_ = INITIALIZING;
      }
    }
    else
    {
      // The current frame
      c_frame_ = Frame(l_img, r_img, camera_model_, timestamp);

      // Publish stereo matches
      f_pub_->publishStereoMatches(c_frame_);

      // Get the pose of the last frame id
      tf::Transform last_frame_pose;
      bool graph_ready = graph_->getFramePose(frame_id_ - 1, last_frame_pose);
      if (!graph_ready) return;

      // Previous/current frame odometry difference
      tf::Transform c_camera_odom_pose = c_odom_robot * odom2camera_;
      tf::Transform odom_diff = odom_pose_history_[odom_pose_history_.size()-1].inverse() * c_camera_odom_pose;

      // Refine its position relative to the previous frame
      tf::Transform correction;
      tf::Transform c_camera_pose;
      int num_inliers = 0;
      if (params_.refine)
      {
        cv::Mat sigma;
        tf::Transform p2c_diff;
        bool succeed = refinePose(p_frame_, c_frame_, p2c_diff, sigma, num_inliers);
        double error = Tools::poseDiff3D(p2c_diff, odom_diff);
        bool refine_valid = succeed && error < 0.3;

        if (refine_valid)
        {
          ROS_INFO_STREAM("[Localization:] Pose refine successful, error: " << error << ", inliers: " << num_inliers);

          c_frame_.setInliersNumWithPreviousFrame(num_inliers);
          c_frame_.setSigmaWithPreviousFrame(sigma);
          correction = p2c_diff;
        }
        else
        {
          ROS_WARN_STREAM("[Localization:] Pose refine fails, error: " << error);
          correction = odom_diff;
        }
      }
      else
      {
        correction = odom_diff;
      }
      c_camera_pose = last_frame_pose * correction;

      // Set frame data
      c_frame_.setCameraPose(c_camera_pose);

      // Need new keyframe
      bool is_new_keyframe = needNewKeyFrame();
      if (is_new_keyframe)
      {
        // Store the camera odometry for this keyframe
        tf::Transform c_odom_camera = c_odom_robot * odom2camera_;
        odom_pose_history_.push_back(c_odom_camera);
      }
    }

    // Convert camera to robot pose
    tf::Transform robot_pose = c_frame_.getCameraPose() * odom2camera_.inverse();

    // Detect a big jump
    double jump = Tools::poseDiff3D(robot_pose, prev_robot_pose_);
    if (!jump_detected_ && jump > 0.8)
    {
      jump_time_ = ros::WallTime::now();
      jump_detected_ = true;
    }
    if (jump_detected_ && ( ros::WallTime::now().toSec() - jump_time_.toSec() > secs_to_filter_ )    )
    {
      jump_detected_ = false;
    }

    tf::Transform pose = robot_pose;
    if (jump_detected_)
    {
      // Filter big jumps
      double m = 1/(10*secs_to_filter_);
      double factor = m * (ros::WallTime::now().toSec() - jump_time_.toSec());

      double c_x = pose.getOrigin().x();
      double c_y = pose.getOrigin().y();
      double c_z = pose.getOrigin().z();

      double p_x = prev_robot_pose_.getOrigin().x();
      double p_y = prev_robot_pose_.getOrigin().y();
      double p_z = prev_robot_pose_.getOrigin().z();

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
    tf_broadcaster_.sendTransform(tf::StampedTransform(pose, odom_msg->header.stamp, "map", odom_msg->child_frame_id));


    // Store
    prev_robot_pose_ = pose;
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

  bool Tracking::needNewKeyFrame()
  {
    // Init initialization
    if (state_ == INITIALIZING)
    {
      bool valid_frame = addFrameToMap();
      if (valid_frame)
        state_ = WORKING;
      return valid_frame;
    }
    else
    {
      // Check odometry distance
      double pose_diff = Tools::poseDiff2D(p_frame_.getCameraPose(), c_frame_.getCameraPose());
      if (pose_diff > params_.dist_keyframes)
      {
        return addFrameToMap();
      }
    }
    return false;
  }

  bool Tracking::addFrameToMap()
  {

    if (c_frame_.getLeftKp().size() > params_.lc_min_inliers)
    {
      c_frame_.regionClustering();

      // Check if enough clusters
      vector< vector<int> > clusters = c_frame_.getClusters();
      if (clusters.size() > 0)
      {
        // Add to graph
        c_frame_.setId(frame_id_);
        f_pub_->publishClustering(c_frame_);
        graph_->addFrameToQueue(c_frame_);

        // Store previous frame
        p_frame_ = c_frame_;

        ROS_INFO_STREAM("[Localization:] Adding keyframe " << frame_id_ + 1);

        // Save the keyframe
        // Increase the frame id counter
        frame_id_++;
        return true;
      }
      else
      {
        ROS_WARN("[Localization:] Image produces 0 clusters.");
        return false;
      }
    }

    ROS_WARN_STREAM("[Localization:] Not enough keypoints in this frame (" << c_frame_.getLeftKp().size() << ")");
    return false;
  }

  bool Tracking::refinePose(Frame query, Frame candidate, tf::Transform& out, cv::Mat& sigma, int& num_inliers)
  {
    // Init
    out.setIdentity();
    num_inliers = params_.lc_min_inliers;

    // Sanity check
    if (query.getLeftDesc().rows == 0 || candidate.getLeftDesc().rows == 0)
      return false;

    // Match current and previous left descriptors
    vector<cv::DMatch> matches;
    Tools::ratioMatching(query.getLeftDesc(), candidate.getLeftDesc(), 0.8, matches);

    if (matches.size() >= params_.lc_min_inliers)
    {
      // Get the matched keypoints
      vector<cv::KeyPoint> query_kp_l = query.getLeftKp();
      vector<cv::KeyPoint> query_kp_r = query.getRightKp();
      vector<cv::Point3f> query_3d = query.getCameraPoints();
      vector<cv::KeyPoint> cand_kp_l = candidate.getLeftKp();
      vector<cv::KeyPoint> cand_kp_r = candidate.getRightKp();
      vector<cv::Point3f> cand_3d = candidate.getCameraPoints();
      vector<cv::Point2f> query_matched_kp_l, query_matched_kp_r;
      vector<cv::Point2f> cand_matched_kp_l, cand_matched_kp_r;
      vector<cv::Point3f> cand_matched_3d_points;
      for(uint i=0; i<matches.size(); i++)
      {
        // Query keypoints
        query_matched_kp_l.push_back(query_kp_l[matches[i].queryIdx].pt);
        query_matched_kp_r.push_back(query_kp_r[matches[i].queryIdx].pt);

        // Candidate keypoints
        cand_matched_kp_l.push_back(cand_kp_l[matches[i].trainIdx].pt);
        cand_matched_kp_r.push_back(cand_kp_r[matches[i].trainIdx].pt);

        // 3d points
        cand_matched_3d_points.push_back(cand_3d[matches[i].trainIdx]);
      }

      cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
      cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
      vector<int> inliers;
      cv::solvePnPRansac(cand_matched_3d_points, query_matched_kp_l, camera_matrix_,
                         cv::Mat(), rvec, tvec, false,
                         100, params_.lc_epipolar_thresh, 0.99, inliers, cv::SOLVEPNP_ITERATIVE);

      // Inliers threshold
      if (inliers.size() < params_.lc_min_inliers)
      {
        return false;
      }
      else
      {
        // Build output transform
        out = Tools::buildTransformation(rvec, tvec);

        // Estimate the covariance
        cv::Mat J;
        vector<cv::Point2f> p;
        vector<cv::Point3f> inliers_3d_points;
        for (uint i=0; i<inliers.size(); i++)
          inliers_3d_points.push_back(cand_matched_3d_points[inliers[i]]);
        cv::projectPoints(inliers_3d_points, rvec, tvec, camera_matrix_, cv::Mat(), p, J);
        cv::Mat tmp = cv::Mat(J.t() * J, cv::Rect(0,0,6,6)).inv();
        cv::sqrt(cv::abs(tmp), sigma);

        // Save the inliers
        num_inliers = inliers.size();
        return true;
      }
    }
    else
    {
      return false;
    }
  }

} //namespace slam
