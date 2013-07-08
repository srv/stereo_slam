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

namespace stereo_slam
{

class Utils
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

  /** \brief get the pose of vertex in format tf::Transform
    * @return tf::Transform pose matrix
    * \param vertex of type g2o::VertexSE3
    */
  static tf::Transform getVertexPose(g2o::VertexSE3* v)
  {
  	Eigen::Isometry3d pose_eigen = v->estimate();
  	tf::Transform pose_tf = stereo_slam::Utils::eigenToTf(pose_eigen);
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

  /** \brief seach correspondences between vertices and false candidates
    * @return true if vertices have been found in false candidates vector, false otherwise
    * \param false_candidates vector with the ids of false candidates
    * \param id_i id of vertex i
    * \param id_j id of vertex j
    */
  static bool searchFalseCandidates(std::vector<cv::Point2i> false_candidates, int id_i, int id_j)
	{
		bool found = false;
	  for (unsigned int i=0; i<false_candidates.size(); i++)
	  {
	  	cv::Point2i vert = false_candidates[i];
	  	if ( (vert.x == id_i && vert.y == id_j) ||
	  			 (vert.x == id_j && vert.y == id_i) )
	  	{
	  		found = true;
	  		break;
	  	}
	  }
	  return found;
	}

	/** \brief Sort 2 descriptors matchings by distance
	  * @return true if vector 1 is smaller than vector 2
	  * \param descriptor matching 1
	  * \param descriptor matching 2
	  */
	static bool sortDescByDistance(const cv::DMatch& d1, const cv::DMatch& d2)
	{
		return (d1.distance < d2.distance);
	}

	/** \brief Sort 2 descriptors matchings by distance
	  * @return true if vector 1 is smaller than vector 2
	  * \param descriptor matching 1
	  * \param descriptor matching 2
	  */
	static std::vector<cv::DMatch> bucketFeatures(std::vector<cv::DMatch> matches, 
																								std::vector<cv::KeyPoint> kp, 
																								int b_width, 
																								int b_height, 
																								int b_num_feautres)
	{
		// Find max values
	  float x_max = 0;
	  float y_max = 0;
	  for (std::vector<cv::DMatch>::iterator it = matches.begin(); it!=matches.end(); it++)
	  {
	    if (kp[it->queryIdx].pt.x > x_max) x_max = kp[it->queryIdx].pt.x;
	    if (kp[it->queryIdx].pt.y > y_max) y_max = kp[it->queryIdx].pt.y;
	  }

	  // Allocate number of buckets needed
	  int bucket_cols = (int)floor(x_max/b_width) + 1;
	  int bucket_rows = (int)floor(y_max/b_height) + 1;
	  std::vector<cv::DMatch> *buckets = new std::vector<cv::DMatch>[bucket_cols*bucket_rows];

	  // Assign matches to their buckets
	  for (std::vector<cv::DMatch>::iterator it=matches.begin(); it!=matches.end(); it++)
	  {
	    int u = (int)floor(kp[it->queryIdx].pt.x/b_width);
	    int v = (int)floor(kp[it->queryIdx].pt.y/b_height);
	    buckets[v*bucket_cols+u].push_back(*it);
	  }

	  // Refill matches from buckets
	  std::vector<cv::DMatch> output;
	  for (int i=0; i<bucket_cols*bucket_rows; i++)
	  {
	    // Sort descriptors matched by distance
	    std::sort(buckets[i].begin(), buckets[i].end(), stereo_slam::Utils::sortDescByDistance);
	    
	    // Add up to max_features features from this bucket to output
	    int k=0;
	    for (std::vector<cv::DMatch>::iterator it=buckets[i].begin(); it!=buckets[i].end(); it++)
	    {
	      output.push_back(*it);
	      k++;
	      if (k >= b_num_feautres)
	        break;
	    }
	  }
	  return output;
	}

};

} // namespace

#endif // UTILS


