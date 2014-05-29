#ifndef TOOLS
#define TOOLS

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <g2o/types/slam3d/edge_se3.h>
#include <image_geometry/stereo_camera_model.h>
#include <nav_msgs/Odometry.h>

namespace stereo_slam
{

class Tools
{

public:

  /** \brief extract the keypoints of some image
    * @return 
    * \param image the source image
    * \param key_points is the pointer for the resulting image key_points
    */
	static void keypointDetector(const cv::Mat& image, 
		std::vector<cv::KeyPoint>& key_points, std::string type)
	{
		cv::initModule_nonfree();
		cv::Ptr<cv::FeatureDetector> cv_detector;
		cv_detector = cv::FeatureDetector::create(type);
		try
    {
  		cv_detector->detect(image, key_points);
  	}
  	catch (cv::Exception& e)
  	{
  		ROS_WARN("[StereoSlam:] cv_detector exception: %s", e.what());
  	}
	}

  /** \brief extract descriptors of some image
    * @return 
    * \param image the source image
    * \param key_points keypoints of the source image
    * \param descriptors is the pointer for the resulting image descriptors
    */
	static void descriptorExtraction(const cv::Mat& image,
	 std::vector<cv::KeyPoint>& key_points, cv::Mat& descriptors, std::string type)
	{
	  cv::Ptr<cv::DescriptorExtractor> cv_extractor;
	  cv_extractor = cv::DescriptorExtractor::create(type);
	  try
	  {
	  	cv_extractor->compute(image, key_points, descriptors);
	  }
	  catch (cv::Exception& e)
	  {
	  	ROS_WARN("[StereoSlam:] cv_extractor exception: %s", e.what());
	  }
	}

  /** \brief match descriptors of 2 images by threshold
    * @return 
    * \param descriptors1 descriptors of image1
    * \param descriptors2 descriptors of image2
    * \param matches between both descriptors
    * \param matching_threshold threshold to determine correct matchings
    */
	static void thresholdMatching(const cv::Mat& descriptors1, 
		const cv::Mat& descriptors2, std::vector<cv::DMatch>& matches, 
		double matching_threshold)
	{
		std::vector<std::vector<cv::DMatch> > matches12;

		cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;
		descriptorMatcher = cv::DescriptorMatcher::create("FlannBased");

	  matches.clear();
	  matches12.clear();
	  int knn = 2;
	  try
	  {
	  	descriptorMatcher->knnMatch(descriptors1, descriptors2, matches12, knn);
	  	for( size_t m = 0; m < matches12.size(); m++ )
	  	{
	  	  if (matches12[m].size() == 1)
	  	  {
	  	    matches.push_back(matches12[m][0]);
	  	  }
	  	  else if (matches12[m].size() == 2) // normal case
	  	  {
	  	    if (matches12[m][0].distance / matches12[m][1].distance 
	  	        < matching_threshold)
	  	    {
	  	      matches.push_back(matches12[m][0]);
	  	    }
	  	  }
	  	}
	  }
	  catch (cv::Exception& e)
	  {
	  	ROS_WARN("[StereoSlam:] cv::DescriptorMatcher exception: %s", e.what());
	  }	  
	}

	/** \brief Compute the 3D point projecting the disparity
	  * @return
	  * \param stereo_camera_model is the camera model
	  * \param left_point on the left image
	  * \param right_point on the right image
	  * \param world_point pointer to the corresponding 3d point
	  */
	static void calculate3DPoint(const image_geometry::StereoCameraModel stereo_camera_model,
								 const cv::Point2d& left_point, 
								 const cv::Point2d& right_point, 
								 cv::Point3d& world_point)
	{
	  double disparity = left_point.x - right_point.x;
	  stereo_camera_model.projectDisparityTo3d(left_point, disparity, world_point);
	}

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
    tf::Vector3 tf_trans( odom_msg.pose.pose.position.x,
                          odom_msg.pose.pose.position.y,
                          odom_msg.pose.pose.position.z);
    tf::Quaternion tf_q ( odom_msg.pose.pose.orientation.x,
                          odom_msg.pose.pose.orientation.y,
                          odom_msg.pose.pose.orientation.z,
                          odom_msg.pose.pose.orientation.w);
    tf::Transform odom(tf_q, tf_trans);
    return odom;
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

  /** \brief compose the transformation matrix using 2 cv::Mat as inputs:
  	* one for rotation and one for translation
    * @return the trasnformation matrix
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
	  double angle = cv::norm(rvec);
	  tf::Quaternion quaternion(axis, angle);

	  tf::Vector3 translation(tvec.at<double>(0, 0), tvec.at<double>(1, 0), 
	      tvec.at<double>(2, 0));

	  return tf::Transform(quaternion, translation);
	}
};

} // namespace

#endif // TOOLS


