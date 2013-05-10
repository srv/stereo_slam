#ifndef UTILS
#define UTILS

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Float32.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <g2o/types/slam3d/edge_se3.h>
#include <image_geometry/stereo_camera_model.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace stereo_localization
{

class Utils
{

public:
	
  /** \brief Compute the centroid of a point cloud
  	* @return a vector with the x,y,z centroid
    * \param point_cloud the source PointCloud
    * \param min_x minimum x value to take into account
    * \param max_x maximum x value to take into account
    * \param min_y minimum y value to take into account
    * \param max_y maximum y value to take into account
    * \param min_z minimum z value to take into account
    * \param max_z maximum z value to take into account
    */
  static tf::Vector3 computeCentroid(PointCloud::Ptr point_cloud, double min_x, double max_x,
	double min_y, double max_y, double min_z, double max_z)
	{
	  double mean_x, mean_y, mean_z;
	  mean_x = 0.0;
	  mean_y = 0.0;
	  mean_z = 0.0;

		int count = 0;
		for (size_t i = 0; i < point_cloud->points.size(); ++i)
		{
		  const pcl::PointXYZ& point = point_cloud->points[i];
		  if (point.x >= min_x && point.x <= max_x &&
		      point.y >= min_y && point.y <= max_y &&
		      point.z >= min_z && point.z <= max_z &&
		      !std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z))
		  {
		    mean_x += point.x;
		    mean_y += point.y;
		    mean_z += point.z;
		    count++;
		  }
		}
		if (count == 0)
		{
		  mean_x = -1;
		  mean_y = -1;
		  mean_z = -1;
		}
		else
		{
		  mean_x /= count;
		  mean_y /= count;
		  mean_z /= count;
		}
		 
		tf::Vector3 centroid = tf::Vector3(mean_x, mean_y, mean_z);

		return centroid;
	}

  /** \brief extract the keypoints of some image
    * @return 
    * \param image the source image
    * \param key_points is the pointer for the resulting image key_points
    */
	static void keypointDetector(const cv::Mat& image, std::vector<cv::KeyPoint>& key_points)
	{
		cv::initModule_nonfree();
		cv::Ptr<cv::FeatureDetector> cv_detector;
		cv_detector = cv::FeatureDetector::create("SIFT");
  	cv_detector->detect(image, key_points);
	}

  /** \brief extract the sift descriptors of some image
    * @return 
    * \param image the source image
    * \param key_points keypoints of the source image
    * \param descriptors is the pointer for the resulting image descriptors
    */
	static void descriptorExtraction(const cv::Mat& image,
	 std::vector<cv::KeyPoint>& key_points, cv::Mat& descriptors)
	{
	  cv::Ptr<cv::DescriptorExtractor> cv_extractor;
	  cv_extractor = cv::DescriptorExtractor::create("SIFT");
	  cv_extractor->compute(image, key_points, descriptors);
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

	/** \brief convert a matrix of type cv::Mat to std::vector
	  * @return std::vector matrix
	  * \param input of type cv::Mat
	  */
	static void calculate3DPoint(	const image_geometry::StereoCameraModel stereo_camera_model,
																const cv::Point2d& left_point, 
																const cv::Point2d& right_point, 
																cv::Point3d& world_point)
	{
	  double disparity = left_point.x - right_point.x;
	  stereo_camera_model.projectDisparityTo3d(left_point, disparity, world_point);
	}

	  /** \brief convert a matrix of type std::vector<cv::KeyPoint> to std::vector
	    * @return std::vector< std::vector<float> > matrix
	    * \param input of type std::vector<cv::KeyPoint>
	    */
		static std::vector< std::vector<float> > cvPoint2fToStdMatrix(std::vector<cv::Point2f> input)
		{
			std::vector< std::vector<float> > output;
			for (unsigned int i=0; i<input.size(); i++)
			{
			  cv::Point2f point = input[i];
			  std::vector<float> p_std(2);
			  p_std[0] = point.x;
			  p_std[1] = point.y;
			  output.push_back(p_std);
			}
			return output;
		}

	  /** \brief convert a matrix of type std::vector to std::vector<cv::Point2f>
	    * @return std::vector<cv::Point2f> matrix
	    * \param input of type std::vector
	    */
		static std::vector<cv::Point2f>stdMatrixToCvPoint2f(std::vector< std::vector<float> > input)
		{
			std::vector<cv::Point2f> output;
			for (unsigned int i=0; i<input.size(); i++)
			{
				std::vector<float> point = input[i];
			  cv::Point2f p_cv;
			  p_cv.x = point[0];
			  p_cv.y = point[1];
			  output.push_back(p_cv);
			}
			return output;
		}

  /** \brief convert a matrix of type std::vector<cv::Point3f> to std::vector
    * @return std::vector< std::vector<float> > matrix
    * \param input of type std::vector<cv::Point3f>
    */
	static std::vector< std::vector<float> > cvPoint3fToStdMatrix(std::vector<cv::Point3f> input)
	{
		std::vector< std::vector<float> > output;
		for (unsigned int i=0; i<input.size(); i++)
		{
		  cv::Point3f point = input[i];
		  std::vector<float> p_std(3);
		  p_std[0] = point.x;
		  p_std[1] = point.y;
		  p_std[2] = point.z;
		  output.push_back(p_std);
		}
		return output;
	}

  /** \brief convert a matrix of type std::vector to std::vector<cv::Point3f>
    * @return std::vector<cv::Point3f> matrix
    * \param input of type std::vector
    */
	static std::vector<cv::Point3f>stdMatrixToCvPoint3f(std::vector< std::vector<float> > input)
	{
		std::vector<cv::Point3f> output;
		for (unsigned int i=0; i<input.size(); i++)
		{
			std::vector<float> point = input[i];
		  cv::Point3f p_cv;
		  p_cv.x = point[0];
		  p_cv.y = point[1];
		  p_cv.z = point[2];
		  output.push_back(p_cv);
		}
		return output;
	}

  /** \brief convert a matrix of type cv::Mat to std::vector
    * @return std::vector matrix
    * \param input of type cv::Mat
    */
	static std::vector< std::vector<float> > cvMatToStdMatrix(cv::Mat input)
	{
		int round_zeros = 1000000000;

		std::vector< std::vector<float> > output;
		for (int i=0; i<input.rows; i++)
		{
		  std::vector<float> vec;
		  for (int j=0; j<input.cols; j++)
		  {
		  	float val = round((float)input.at<float>(i,j)*round_zeros)/round_zeros;
		    vec.push_back(val);
		  }
		  output.push_back(vec);
		}
		return output;
	}

  /** \brief convert a matrix of type std::vector to cv::Mat
    * @return cv::Mat matrix
    * \param input of type std::vector< std::vector<float> >
    */
	static cv::Mat stdMatrixToCvMat(std::vector< std::vector<float> > input)
	{
		if (input.size() > 0)
		{
			std::vector<float> row0 = input[0];
			cv::Mat output(input.size(), row0.size(), cv::DataType<float>::type);

			for (unsigned int i=0; i<input.size(); i++)
			{
				std::vector<float> row = (std::vector<float>)input[i];
				for (unsigned int j=0; j<row.size(); j++)
				{
					output.at<float>(i,j) = (float)row[j];
				}
			}
			return output;
		}
		else
		{
			cv::Mat empty;
			return empty;
		}
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

  /** \brief get the pose of node in format tf::Transform
    * @return tf::Transform pose matrix
    * \param node of type g2o::VertexSE3
    */
  static tf::Transform getNodePose(g2o::VertexSE3* node)
  {
  	Eigen::Isometry3d pose_eigen = node->estimate();
  	tf::Transform pose_tf = stereo_localization::Utils::eigenToTf(pose_eigen);
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

	  tf::Vector3 axis(rvec.at<double>(0, 0), rvec.at<double>(1, 0), 
	      rvec.at<double>(2, 0));
	  double angle = cv::norm(rvec);
	  tf::Quaternion quaternion(axis, angle);

	  tf::Vector3 translation(tvec.at<double>(0, 0), tvec.at<double>(1, 0), 
	      tvec.at<double>(2, 0));

	  return tf::Transform(quaternion, translation);
	}

  /** \brief seach correspondences between nodes and false candidates
    * @return true if nodes have been found in false candidates vector, false otherwise
    * \param false_candidates vector with the ids of false candidates
    * \param id_i id of node i
    * \param id_j id of node j
    */
  static bool searchFalseCandidates(std::vector<cv::Point2i> false_candidates, int id_i, int id_j)
	{
		bool found = false;
	  for (unsigned int i=0; i<false_candidates.size(); i++)
	  {
	  	cv::Point2i nodes = false_candidates[i];
	  	if ( (nodes.x == id_i && nodes.y == id_j) ||
	  			 (nodes.x == id_j && nodes.y == id_i) )
	  	{
	  		found = true;
	  		break;
	  	}
	  }
	  return found;
	}
};

} // namespace

#endif // UTILS


