#include <ros/ros.h>

#include "tracking.h"
#include "tools.h"

using namespace tools;

namespace slam
{

  Tracking::Tracking(Publisher *f_pub, Graph *graph)
                    : f_pub_(f_pub), graph_(graph)
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

    // Message sync
    boost::shared_ptr<Sync> sync;
    odom_sub      .subscribe(nh, params_.odom_topic, 20);
    left_sub      .subscribe(it, params_.camera_topic+"/left/image_rect_color", 3);
    right_sub     .subscribe(it, params_.camera_topic+"/right/image_rect_color", 3);
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

      state_ = INITIALIZING;
    }
    else
    {
      // The current frame
      c_frame_ = Frame(l_img, r_img, camera_model_, timestamp);

      // Set the odometry of this frame
      tf::Transform c_odom_camera = c_odom_robot * odom2camera_;
      c_frame_.setCameraPose(c_odom_camera);

      // Need new keyframe
      needNewKeyFrame();
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

  void Tracking::needNewKeyFrame()
  {
    // Init initialization
    if (state_ == INITIALIZING)
    {
      addFrameToMap(c_frame_);
      state_ = WORKING;
    }
    else
    {
      // Do not add very close frames
      double pose_diff = Tools::poseDiff(last_fixed_frame_pose_, c_frame_.getCameraPose());
      if (pose_diff > 0.10)
      {
        addFrameToMap(c_frame_);
      }
    }
  }

  void Tracking::addFrameToMap(Frame frame)
  {
    if (frame.getLeftKp().size() > LC_MIN_INLIERS)
    {
      frame.regionClustering();
      f_pub_->publishClustering(frame);
      graph_->addFrameToQueue(frame);
      last_fixed_frame_pose_ = frame.getCameraPose();
    }
  }

} //namespace slam