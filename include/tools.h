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


  /** \brief convert a Eigen::Vector4f to tf::Transform
    * @return tf::Transform matrix
    * \param in of type Eigen::Vector4f
    */
  static tf::Transform vector4fToTransform(Eigen::Vector4f in)
  {
    tf::Transform out;
    out.setIdentity();
    tf::Vector3 t_out(in[0], in[1], in[2]);
    out.setOrigin(t_out);
    return out;
  }

  /** \brief Transform vector4f
    * @return transformed Eigen::Vector4f
    * \param input Eigen::Vector4f
    * \param input transformation
    */
  static tf::Transform transformVector4f(Eigen::Vector4f in, tf::Transform transform)
  {
    tf::Transform in_tf = vector4fToTransform(in);
    return transform * in_tf;
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

  /** \brief compute the absolute diference between 2 poses (omitting z)
    * @return the norm between two poses
    * \param pose_1 transformation matrix of pose 1
    * \param pose_2 transformation matrix of pose 2
    */
  static double poseDiff3D(tf::Transform pose_1, tf::Transform pose_2)
  {
    tf::Vector3 d = pose_1.getOrigin() - pose_2.getOrigin();
    return sqrt(d.x()*d.x() + d.y()*d.y() + d.z()*d.z());
  }

  /** \brief compute the absolute diference between 2 poses (omitting z)
    * @return the norm between two poses
    * \param pose_1 transformation matrix of pose 1
    * \param pose_2 transformation matrix of pose 2
    */
  static double poseDiff2D(tf::Transform pose_1, tf::Transform pose_2)
  {
    tf::Vector3 d = pose_1.getOrigin() - pose_2.getOrigin();
    return sqrt(d.x()*d.x() + d.y()*d.y());
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

  /** \brief Sort 2 pairs by size
   * @return true if pair 1 is smaller than pair 2
   * \param pair 1
   * \param pair 2
   */
 static bool sortByDistance(const pair<int, double> d1, const pair<int, double> d2)
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

  static cv::Point3f transformPoint(cv::Point3f point, tf::Transform base)
  {
    tf::Vector3 p_tf(point.x, point.y, point.z);
    tf::Vector3 p_tf_world = base * p_tf;
    cv::Point3f new_point(p_tf_world.x(), p_tf_world.y(), p_tf_world.z());
    return new_point;
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
    if (desc_1.rows < 10 || desc_2.rows < 10) return;

    assert(desc_1.type() == desc_2.type());
    assert(desc_1.cols == desc_2.cols);

    cv::Mat match_mask;
    const int knn = 2;
    cv::Ptr<cv::DescriptorMatcher> descriptor_matcher;
    if (desc_1.type() == CV_8U)
      descriptor_matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    else
      descriptor_matcher = cv::DescriptorMatcher::create("BruteForce");
    vector<vector<cv::DMatch> > knn_matches;
    descriptor_matcher->knnMatch(desc_1, desc_2, knn_matches, knn, match_mask);
    for (uint m=0; m<knn_matches.size(); m++)
    {
      if (knn_matches[m].size() < 2) continue;
      if (knn_matches[m][0].distance <= knn_matches[m][1].distance * ratio)
        matches.push_back(knn_matches[m][0]);
    }
  }

  /** \brief match descriptors of 2 images by threshold
    * @return
    * \param descriptors1 descriptors of image1
    * \param descriptors2 descriptors of image2
    * \param threshold to determine correct matchings
    * \param match_mask mask for matchings
    * \param matches output vector with the matches
    */
  static void thresholdMatching(const cv::Mat& descriptors1, const cv::Mat& descriptors2,
    double threshold, const cv::Mat& match_mask, vector<cv::DMatch>& matches)
  {
    matches.clear();
    if (descriptors1.empty() || descriptors2.empty())
      return;
    assert(descriptors1.type() == descriptors2.type());
    assert(descriptors1.cols == descriptors2.cols);

    const int knn = 2;
    cv::Ptr<cv::DescriptorMatcher> descriptor_matcher;
    // choose matcher based on feature type
    if (descriptors1.type() == CV_8U)
      descriptor_matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    else
      descriptor_matcher = cv::DescriptorMatcher::create("BruteForce");
    vector<vector<cv::DMatch> > knn_matches;
    descriptor_matcher->knnMatch(descriptors1, descriptors2,
            knn_matches, knn, match_mask);

    for (size_t m = 0; m < knn_matches.size(); m++)
    {
      if (knn_matches[m].size() < 2) continue;
      float dist1 = knn_matches[m][0].distance;
      float dist2 = knn_matches[m][1].distance;
      if (dist1 / dist2 < threshold)
      {
        matches.push_back(knn_matches[m][0]);
      }
    }
  }

  /** \brief filter matches of cross check matching
    * @return
    * \param matches1to2 matches from image 1 to 2
    * \param matches2to1 matches from image 2 to 1
    * \param matches output vector with filtered matches
    */
  static void crossCheckFilter(
      const vector<cv::DMatch>& matches1to2,
      const vector<cv::DMatch>& matches2to1,
      vector<cv::DMatch>& checked_matches)
  {
    checked_matches.clear();
    for (size_t i = 0; i < matches1to2.size(); ++i)
    {
      bool match_found = false;
      const cv::DMatch& forward_match = matches1to2[i];
      for (size_t j = 0; j < matches2to1.size() && match_found == false; ++j)
      {
        const cv::DMatch& backward_match = matches2to1[j];
        if (forward_match.trainIdx == backward_match.queryIdx &&
            forward_match.queryIdx == backward_match.trainIdx)
        {
          checked_matches.push_back(forward_match);
          match_found = true;
        }
      }
    }
  }

  /** \brief match descriptors of 2 images by threshold
    * @return
    * \param descriptors1 descriptors of image 1
    * \param descriptors2 descriptors of image 2
    * \param threshold to determine correct matchings
    * \param match_mask mask for matchings
    * \param matches output vector with the matches
    */
  static void crossCheckThresholdMatching(
    const cv::Mat& descriptors1, const cv::Mat& descriptors2,
    double threshold, vector<cv::DMatch>& matches)
  {
    const cv::Mat match_mask;

    vector<cv::DMatch> query_to_train_matches;
    thresholdMatching(descriptors1, descriptors2, threshold, match_mask, query_to_train_matches);
    vector<cv::DMatch> train_to_query_matches;
    cv::Mat match_mask_t;
    if (!match_mask.empty()) match_mask_t = match_mask.t();
    thresholdMatching(descriptors2, descriptors1, threshold, match_mask_t, train_to_query_matches);

    crossCheckFilter(query_to_train_matches, train_to_query_matches, matches);
  }

  static string convertTo5digits(int in)
  {
    uint val = (uint)in;
    string output;
    int digits = 5;
    while (digits-- > 0) {
        output += ('0' + val % 10);
        val /= 10;
    }
    reverse(output.begin(), output.end());
    return output;
  }
};

} // namespace

#endif // TOOLS