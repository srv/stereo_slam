#include "stereo_slam_base.h"
#include <boost/shared_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <libpq-fe.h>
#include <Eigen/Geometry>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "postgresql_interface.h"
#include "utils.h"

/** \brief Messages callback. This function is called when syncronized odometry and image
  * message are received.
  * @return 
  * \param l_ptr pointer to left image
  * \param r_ptr pointer to right image
  * \param current_pose from visual odometry
  * \param corrected_pose pose corrected by the graph
  * \param timestamp message timestamp
  */
void stereo_slam::StereoSlamBase::vertexInsertion(cv_bridge::CvImagePtr l_ptr, 
                                                  cv_bridge::CvImagePtr r_ptr,
                                                  tf::Transform current_pose,
                                                  tf::Transform corrected_pose,
                                                  double timestamp)
{
  // Extract keypoints and descriptors of images
  std::vector<cv::KeyPoint> l_kp, r_kp;
  cv::Mat l_desc = cv::Mat_<std::vector<float> >();
  cv::Mat r_desc = cv::Mat_<std::vector<float> >();
  stereo_slam::Utils::keypointDetector(l_ptr->image, l_kp, "SIFT");
  stereo_slam::Utils::keypointDetector(r_ptr->image, r_kp, "SIFT");
  stereo_slam::Utils::descriptorExtraction(l_ptr->image, l_kp, l_desc, "SIFT");
  stereo_slam::Utils::descriptorExtraction(r_ptr->image, r_kp, r_desc, "SIFT");

  // Find matching between stereo images
  std::vector<cv::DMatch> matches;
  stereo_slam::Utils::thresholdMatching(l_desc, r_desc, matches, params_.descriptor_threshold);

  // Compute 3D points
  std::vector<cv::Point2f> matched_keypoints;
  std::vector<cv::Point3f> matched_3d_points;
  cv::Mat matched_descriptors;
  for (size_t i = 0; i < matches.size(); ++i)
  {
    int index_left = matches[i].queryIdx;
    int index_right = matches[i].trainIdx;

    if (abs(l_kp[index_left].pt.y - r_kp[index_right].pt.y) < params_.epipolar_threshold)
    {
      cv::Point3d world_point;
      stereo_slam::Utils::calculate3DPoint( stereo_camera_model_,
                                            l_kp[index_left].pt,
                                            r_kp[index_right].pt,
                                            world_point);
      matched_3d_points.push_back(world_point);
      matched_keypoints.push_back(l_kp[index_left].pt);
      matched_descriptors.push_back(l_desc.row(index_left));
    }
  }

  if (params_.stereo_vision_verbose)
    ROS_INFO_STREAM("[StereoSlam:] Found " << matches.size() <<
     " matches between left-right pair. " << matched_3d_points.size() <<
     " after epipolar filtering.");

  // Transform data to std::vector for database
  std::vector< std::vector<float> > keypoints = 
  stereo_slam::Utils::cvPoint2fToStdMatrix(matched_keypoints);
  std::vector< std::vector<float> > descriptors = 
  stereo_slam::Utils::cvMatToStdMatrix(matched_descriptors);
  std::vector< std::vector<float> > points_3d = 
  stereo_slam::Utils::cvPoint3fToStdMatrix(matched_3d_points);

  // Save vertex data into the database
  stereo_slam::GraphDB vertex_data;
  vertex_data.keypoints_.data() = keypoints;
  vertex_data.descriptors_.data() = descriptors;
  vertex_data.points3d_.data() = points_3d;
  if (!pg_db_ptr_thread_1_->insertIntoDatabase(&vertex_data))
  {
    ROS_ERROR("[StereoSlam:] Vertex insertion failed");
  }
  else
  {
    // Everything is ok, save the vertex into the graph

    // Build the pose
    Eigen::Isometry3d pose = stereo_slam::Utils::tfToEigen(corrected_pose);

    // Build the vertex
    g2o::VertexSE3* cur_vertex = new g2o::VertexSE3();
    cur_vertex->setId(vertex_data.id_.data() - 1);
    cur_vertex->setEstimate(pose);
    if (first_vertex_)
    {
      // First time, no edges.
      cur_vertex->setFixed(true);
      graph_optimizer_.addVertex(cur_vertex);
      first_vertex_ = false;
    }
    else
    {
      // When graph has been initialized get the transform between current and previous vertices
      // and save it as an edge

      // Get last vertex
      g2o::VertexSE3* prev_vertex = dynamic_cast<g2o::VertexSE3*>(
        graph_optimizer_.vertices()[graph_optimizer_.vertices().size() - 1]);
      graph_optimizer_.addVertex(cur_vertex);

      // Odometry edges
      g2o::EdgeSE3* e = new g2o::EdgeSE3();
      Eigen::Isometry3d t = prev_vertex->estimate().inverse() * cur_vertex->estimate();
      e->setVertex(0, prev_vertex);
      e->setVertex(1, cur_vertex);
      e->setMeasurement(t);
      graph_optimizer_.addEdge(e);
    }

    // Save original pose history
    pose_history_.push_back(current_pose);
    pose_history_stamp_.push_back(timestamp);

    // Save for next iteration
    previous_pose_ = corrected_pose;
    first_message_ = false;
    ROS_INFO_STREAM("[StereoSlam:] Vertex " << vertex_data.id_.data() << " insertion done");
  }
}