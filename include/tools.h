#ifndef TOOLS
#define TOOLS

#include <ros/ros.h>
#include <fstream>
#include <pcl/common/common.h>
#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <boost/filesystem.hpp>
#include <g2o/types/slam3d/vertex_se3.h>

namespace enc = sensor_msgs::image_encodings;
namespace fs  = boost::filesystem;

using namespace std;

namespace tools
{

class Tools
{

public:

  // Definitions
  typedef pcl::PointXYZRGB        Point;
  typedef pcl::PointCloud<Point>  PointCloud;

  /** \brief convert a tf::transform to Eigen::Isometry3d
    * @return Eigen::Isometry3d matrix
    * \param in of type tf::transform
    */
  static Eigen::Isometry3d vector4fToIsometry(Eigen::Vector4f in)
  {
    Eigen::Vector3d t_out(in[0], in[1], in[2]);
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.translation() = t_out;
    return out;
  }

  /** \brief convert a tf::transform to Eigen::Isometry3d
    * @return Eigen::Isometry3d matrix
    * \param in of type tf::transform
    */
  static Eigen::Isometry3d tfToIsometry(tf::Transform in)
  {
    tf::Vector3 t_in = in.getOrigin();
    tf::Quaternion q_in = in.getRotation();
    Eigen::Vector3d t_out(t_in.x(), t_in.y(), t_in.z());
    Eigen::Quaterniond q_out;
    q_out.setIdentity();
    q_out.x() = q_in.x();
    q_out.y() = q_in.y();
    q_out.z() = q_in.z();
    q_out.w() = q_in.w();
    Eigen::Isometry3d out = (Eigen::Isometry3d)q_out;
    out.translation() = t_out;
    return out;
  }

  /** \brief convert a Eigen::Isometry3d to tf::transform
    * @return tf::transform matrix
    * \param in of type Eigen::Isometry3d
    */
  static tf::Transform isometryToTf(Eigen::Isometry3d in)
  {
    Eigen::Vector3d t_in = in.translation();
    Eigen::Quaterniond q_in = (Eigen::Quaterniond)in.rotation();
    tf::Vector3 t_out(t_in.x(), t_in.y(), t_in.z());
    tf::Quaternion q_out(q_in.x(), q_in.y(), q_in.z(), q_in.w());
    tf::Transform out(q_out, t_out);
    return out;
  }

  /** \brief Convert odometry message to tf::Transform
    * @return the trasnformation matrix
    * \param rvec cv matrix with the rotation angles
    * \param tvec cv matrix with the transformation x y z
    */
  static tf::Transform odomTotf(nav_msgs::Odometry odom_msg)
  {
    // Get the data
    double tx = odom_msg.pose.pose.position.x;
    double ty = odom_msg.pose.pose.position.y;
    double tz = odom_msg.pose.pose.position.z;

    double qx = odom_msg.pose.pose.orientation.x;
    double qy = odom_msg.pose.pose.orientation.y;
    double qz = odom_msg.pose.pose.orientation.z;
    double qw = odom_msg.pose.pose.orientation.w;

    // Sanity check
    if(qx == 0.0 && qy == 0.0 && qz == 0.0 && qw == 1.0)
    {
      tf::Transform odom;
      odom.setIdentity();
      return odom;
    }
    else
    {
      tf::Vector3 tf_trans(tx, ty, tz);
      tf::Quaternion tf_q (qx, qy, qz, qw);
      tf::Transform odom(tf_q, tf_trans);
      return odom;
    }
  }

  /** \brief Convert image messages to cv cv::Mat
    * \param left image message.
    * \param right image message.
    * \param will contain the output left cv::Mat.
    * \param will contain the output right cv::Mat.
    */
  static bool imgMsgToMat(sensor_msgs::Image l_img_msg,
                          sensor_msgs::Image r_img_msg,
                          cv::Mat &l_img, cv::Mat &r_img)
  {
    // Convert message to cv::Mat
    cv_bridge::CvImagePtr l_img_ptr, r_img_ptr;
    try
    {
      l_img_ptr = cv_bridge::toCvCopy(l_img_msg, enc::BGR8);
      r_img_ptr = cv_bridge::toCvCopy(r_img_msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("[StereoSlam:] cv_bridge exception: %s", e.what());
      return false;
    }

    // Set the images
    l_img = l_img_ptr->image;
    r_img = r_img_ptr->image;
    return true;
  }

  /** \brief Get the stereo camera model from info messages
    * \param left image info message.
    * \param right image info message.
    * \param will contain the output camera model.
    * \param will contain the output camera matrix.
    */
  static void getCameraModel(sensor_msgs::CameraInfo l_info_msg,
                             sensor_msgs::CameraInfo r_info_msg,
                             image_geometry::StereoCameraModel &stereo_camera_model,
                             cv::Mat &camera_matrix)
  {
    // Get the binning factors
    int binning_x = l_info_msg.binning_x;
    int binning_y = l_info_msg.binning_y;

    // Get the stereo camera model
    stereo_camera_model.fromCameraInfo(l_info_msg, r_info_msg);

    // Get the projection/camera matrix
    const cv::Mat P(3,4, CV_64FC1, const_cast<double*>(l_info_msg.P.data()));
    camera_matrix = P.colRange(cv::Range(0,3)).clone();

    // Are the images scaled?
    if (binning_x > 1 || binning_y > 1)
    {
      camera_matrix.at<double>(0,0) = camera_matrix.at<double>(0,0) / binning_x;
      camera_matrix.at<double>(0,2) = camera_matrix.at<double>(0,2) / binning_x;
      camera_matrix.at<double>(1,1) = camera_matrix.at<double>(1,1) / binning_y;
      camera_matrix.at<double>(1,2) = camera_matrix.at<double>(1,2) / binning_y;
    }
  }

  /** \brief get the pose of vertex in format tf::Transform
    * @return tf::Transform pose matrix
    * \param vertex
    */
  static tf::Transform getVertexPose(g2o::VertexSE3* v)
  {
    Eigen::Isometry3d pose_eigen = v->estimate();
    tf::Transform pose_tf = tools::Tools::isometryToTf(pose_eigen);
    return pose_tf;
  }

  /** \brief Sort 2 matchings by value
    * @return true if matching 1 is smaller than matching 2
    * \param matching 1
    * \param matching 2
    */
  static bool sortByMatching(const pair<int, float> d1, const pair<int, float> d2)
  {
    return (d1.second < d2.second);
  }

  /** \brief compose the transformation matrix using 2 cv::Mat as inputs:
    * one for rotation and one for translation
    * @return the transformation matrix
    * \param rvec cv matrix with the rotation angles
    * \param tvec cv matrix with the transformation x y z
    */
  static tf::Transform buildTransformation(cv::Mat rvec, cv::Mat tvec)
  {
    if (rvec.empty() || tvec.empty())
      return tf::Transform();

    tf::Vector3 axis(rvec.at<double>(0, 0),
               rvec.at<double>(1, 0),
                 rvec.at<double>(2, 0));
    double angle = norm(rvec);
    tf::Quaternion quaternion(axis, angle);

    tf::Vector3 translation(tvec.at<double>(0, 0), tvec.at<double>(1, 0),
        tvec.at<double>(2, 0));

    return tf::Transform(quaternion, translation);
  }

  /** \brief convert tf::Transform to cv::Mat
    * @return the transformation matrix in cv::Mat format
    * \param input tf::Transform
    */
  static cv::Mat transformToMat(tf::Transform in)
  {
    cv::Mat out(4,4,CV_32FC1);
    tf::Matrix3x3 rot = in.getBasis();
    tf::Vector3 trans = in.getOrigin();
    out.at<double>(0,0) = rot[0][0];
    out.at<double>(0,1) = rot[0][1];
    out.at<double>(0,2) = rot[0][2];
    out.at<double>(0,3) = trans.x();
    out.at<double>(1,0) = rot[1][0];
    out.at<double>(1,1) = rot[1][1];
    out.at<double>(1,2) = rot[1][2];
    out.at<double>(1,3) = trans.y();
    out.at<double>(2,0) = rot[2][0];
    out.at<double>(2,1) = rot[2][1];
    out.at<double>(2,2) = rot[2][2];
    out.at<double>(2,3) = trans.z();
    out.at<double>(2,0) = 0.0;
    out.at<double>(2,1) = 0.0;
    out.at<double>(2,2) = 0.0;
    out.at<double>(2,3) = 1.0;
    return out;
  }

  /** \brief convert cv::Mat to tf::Transform
    * @return the transformation matrix in tf::Transform format
    * \param input cv::Mat
    */
  static tf::Transform matToTransform(cv::Mat in)
  {
    tf::Matrix3x3 rot;
    rot[0][0] = in.at<double>(0,0);
    rot[0][1] = in.at<double>(0,1);
    rot[0][2] = in.at<double>(0,2);
    rot[1][0] = in.at<double>(1,0);
    rot[1][1] = in.at<double>(1,1);
    rot[1][2] = in.at<double>(1,2);
    rot[2][0] = in.at<double>(2,0);
    rot[2][1] = in.at<double>(2,1);
    rot[2][2] = in.at<double>(2,2);
    tf::Vector3 trans(in.at<double>(0,3),
                      in.at<double>(1,3),
                      in.at<double>(2,3));
    tf::Transform out(rot, trans);
    return out;
  }

  /** \brief Ration matching between descriptors
    * \param Descriptors of image 1
    * \param Descriptors of image 2
    * \param ratio value (0.6/0.9)
    * \param output matching
    */
  static void ratioMatching(cv::Mat desc_1, cv::Mat desc_2, double ratio, vector<cv::DMatch> &matches)
  {
    matches.clear();
    cv::Mat match_mask;
    const int knn = 2;
    cv::Ptr<cv::DescriptorMatcher> descriptor_matcher;
    descriptor_matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    vector<vector<cv::DMatch> > knn_matches;
    descriptor_matcher->knnMatch(desc_1, desc_2, knn_matches, knn, match_mask);
    for (uint m=0; m<knn_matches.size(); m++)
    {
      if (knn_matches[m].size() < 2) continue;
      if (knn_matches[m][0].distance <= knn_matches[m][1].distance * ratio)
        matches.push_back(knn_matches[m][0]);
    }
  }
};

} // namespace

#endif // TOOLS