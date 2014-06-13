#ifndef TOOLS
#define TOOLS

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <g2o/types/slam3d/edge_se3.h>
#include <image_geometry/stereo_camera_model.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace cv;

namespace stereo_slam
{

class Tools
{

public:

  /** \brief convert a tf::transform to Eigen::Isometry3d
    * @return Eigen::Isometry3d matrix
    * \param in of type tf::transform
    */
  static Eigen::Isometry3d tfToEigen(tf::Transform in)
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
  static tf::Transform eigenToTf(Eigen::Isometry3d in)
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
    if(qx == 0.0 && qy == 0.0 && qz == 0.0 && qw == 0.0)
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

  /** \brief get the pose of vertex in format tf::Transform
    * @return tf::Transform pose matrix
    * \param vertex of type g2o::VertexSE3
    */
  static tf::Transform getVertexPose(g2o::VertexSE3* v)
  {
    Eigen::Isometry3d pose_eigen = v->estimate();
    tf::Transform pose_tf = stereo_slam::Tools::eigenToTf(pose_eigen);
    return pose_tf;
  }

  /** \brief compute the absolute diference between 2 poses
    * @return the norm between two poses
    * \param pose_1 transformation matrix of pose 1
    * \param pose_2 transformation matrix of pose 2
    */
  static double poseDiff(tf::Transform pose_1, tf::Transform pose_2)
  {
    tf::Vector3 d = pose_1.getOrigin() - pose_2.getOrigin();
    return sqrt(d.x()*d.x() + d.y()*d.y() + d.z()*d.z());
  }

  /** \brief Convert isometry to string (useful for debugin purposes)
    * @return string containing the isometry
    * \param isometry matrix
    */
  static string isometryToString(Eigen::Isometry3d m)
  {
    char result[80];
    memset(result, 0, sizeof(result));
    Eigen::Vector3d xyz = m.translation();
    Eigen::Vector3d rpy = m.rotation().eulerAngles(0, 1, 2);
    snprintf(result, 79, "%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f",
        xyz(0), xyz(1), xyz(2),
        rpy(0) * 180/M_PI, rpy(1) * 180/M_PI, rpy(2) * 180/M_PI);
    return string(result);
  }

  /** \brief Show a tf in the console (useful for debugin purposes)
    * @return
    * \param tf matrix
    */
  static void showTf(tf::Transform input)
  {
    tf::Vector3 tran = input.getOrigin();
    tf::Matrix3x3 rot = input.getBasis();
    tf::Vector3 r0 = rot.getRow(0);
    tf::Vector3 r1 = rot.getRow(1);
    tf::Vector3 r2 = rot.getRow(2);
    ROS_INFO_STREAM("[StereoSlam:]\n" << r0.x() << ", " << r0.y() << ", " << r0.z() << ", " << tran.x() <<
                    "\n" << r1.x() << ", " << r1.y() << ", " << r1.z() << ", " << tran.y() <<
                    "\n" << r2.x() << ", " << r2.y() << ", " << r2.z() << ", " << tran.z());
  }
};

} // namespace

#endif // TOOLS