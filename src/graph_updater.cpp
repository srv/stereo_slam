#include "stereo_slam_base.h"
#include <boost/shared_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <libpq-fe.h>
#include <Eigen/Geometry>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "postgresql_interface.h"
#include "utils.h"

/** \brief Updates the graph
  * @return true if new edges have been inserted into the graph.
  */
bool stereo_slam::StereoSlamBase::graphUpdater()
{
  // True when new edges to force optimization
  bool edge_added = false;

  // Find possible candidates for loop-closing
  unsigned int graph_size = graph_optimizer_.vertices().size();
  for (unsigned int i=0; i<graph_size; i++)
  {
    // Extract the pose of vertex i
    g2o::VertexSE3* v_i = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[i]);
    tf::Transform pose_i = stereo_slam::Utils::getVertexPose(v_i);

    for (unsigned int j=i+params_.neighbor_offset; j<graph_size; j++)
    {
      // Extract the pose of vertex j and compare
      g2o::VertexSE3* v_j = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[j]);
      tf::Transform pose_j = stereo_slam::Utils::getVertexPose(v_j);

      // Check if this have been discarted previously
      bool false_cand = stereo_slam::Utils::searchFalseCandidates(false_candidates_, v_i->id(), v_j->id());
      if (false_cand)
        continue;

      // Spherical ROI
      double pose_diff = stereo_slam::Utils::poseDiff(pose_i, pose_j);
      if (pose_diff > params_.max_candidate_threshold)
      {
        cv::Point2i c(i,j);
        false_candidates_.push_back(c);
        continue;
      }

      // Get the possible tf between nodes
      tf::Transform cl_edge;
      if (!getLoopClosing(v_i, v_j, cl_edge))
      {
        // Discard correspondence and cotinue with the next element of the "for"
        cv::Point2i c(i,j);
        false_candidates_.push_back(c);
        continue;
      }

      // To prevent for possible errors, compare previous transform with the new edge found
      Eigen::Isometry3d t = v_i->estimate().inverse() * v_j->estimate();
      tf::Transform cl_edge_prev = stereo_slam::Utils::eigenToTf(t);

      double edge_diff = stereo_slam::Utils::poseDiff(cl_edge, cl_edge_prev);
      if (params_.stereo_vision_verbose)
        ROS_INFO_STREAM("[StereoSlam:] The error between node " << v_i->id() 
          << " and " << v_j->id() << " is: " << edge_diff << 
          " (max_edge_err is: " << params_.max_edge_err << ")");

      // Check edge geometry
      if (edge_diff > params_.max_edge_err)
      {
        cv::Point2i c(i,j);
        false_candidates_.push_back(c);
        continue;
      }

      // Add the new edge to graph
      g2o::EdgeSE3* e = new g2o::EdgeSE3();
      Eigen::Isometry3d tf_edge = stereo_slam::Utils::tfToEigen(cl_edge);
      e->setVertex(0, v_j);
      e->setVertex(1, v_i);
      e->setMeasurement(tf_edge);
      graph_optimizer_.addEdge(e);
      edge_added = true;

      // Discard this edge for the next iterations
      cv::Point2i c(i,j);
      false_candidates_.push_back(c);

      ROS_INFO("[StereoSlam:]***********************************************");
      ROS_INFO_STREAM("[StereoSlam:] Loop closed between vertices " << v_i->id()+1 << " and " << v_j->id()+1);
      ROS_INFO("[StereoSlam:]***********************************************");
    }
  }

  return edge_added;
}

/** \brief Gets the loop closing between the vertices (if any)
  * @return 
  * \param true if loop closure is available, false otherwise.
  */
bool stereo_slam::StereoSlamBase::getLoopClosing(g2o::VertexSE3* v_i, g2o::VertexSE3* v_j, tf::Transform& output)
{
  // Get the data of both vertices from database
  std::string where_i = "(id = " + boost::lexical_cast<std::string>(v_i->id() + 1) + ")";
  std::string where_j = "(id = " + boost::lexical_cast<std::string>(v_j->id() + 1) + ")";
  std::vector< boost::shared_ptr<stereo_slam::GraphDB> > vert_i;
  std::vector< boost::shared_ptr<stereo_slam::GraphDB> > vert_j;
  pg_db_ptr_thread_2_->getList(vert_i, where_i);
  pg_db_ptr_thread_2_->getList(vert_j, where_j);
  if (vert_i.size() != 1 || vert_j.size() != 1)
    return false;

  cv::Mat desc_i = cv::Mat_<std::vector<float> >();
  cv::Mat desc_j = cv::Mat_<std::vector<float> >();
  desc_i = stereo_slam::Utils::stdMatrixToCvMat(vert_i[0]->descriptors_.data());
  desc_j = stereo_slam::Utils::stdMatrixToCvMat(vert_j[0]->descriptors_.data());

  // Check for enought descriptors
  if (desc_i.rows < params_.matches_threshold || desc_j.rows < params_.matches_threshold)
    return false;

  // Compute matchings
  std::vector<cv::DMatch> matches;
  stereo_slam::Utils::thresholdMatching(desc_i, desc_j, matches, params_.descriptor_threshold);
  
  if (params_.stereo_vision_verbose)
    ROS_INFO_STREAM("[StereoSlam:] Found " << matches.size() <<
       " matches between vertices " << v_i->id()+1 << " and " << v_j->id()+1 <<
       " (matches_threshold is: " << params_.matches_threshold << ")");

  if ((int)matches.size() < params_.matches_threshold)
    return false;

  // Extract keypoints and 3d points of vertex i and j
  std::vector<cv::Point2f> keypoints_j;
  std::vector<cv::Point3f> points3d_i;
  points3d_i = stereo_slam::Utils::stdMatrixToCvPoint3f(vert_i[0]->points3d_.data());
  keypoints_j = stereo_slam::Utils::stdMatrixToCvPoint2f(vert_j[0]->keypoints_.data());
  std::vector<cv::Point2f> matched_points_j;
  std::vector<cv::Point3f> matched_3d_points;
  for (size_t n = 0; n < matches.size(); ++n)
  {
    int index_left = matches[n].queryIdx;
    int index_right = matches[n].trainIdx;;
    matched_3d_points.push_back(points3d_i[index_left]);
    matched_points_j.push_back(keypoints_j[index_right]);
  }
  
  // Compute the transformation between the vertices
  cv::Mat rvec, tvec;
  std::vector<int> inliers;
  cv::solvePnPRansac(matched_3d_points, matched_points_j, camera_matrix_, 
                     cv::Mat(), rvec, tvec, false, 
                     params_.max_solvepnp_iter, params_.allowed_reprojection_err, 
                     params_.max_inliers, inliers);

  if (params_.stereo_vision_verbose)
    ROS_INFO_STREAM("[StereoSlam:] Found " << inliers.size() <<
     " inliers between vertices " << v_i->id()+1 << " and " << v_j->id()+1 <<
     " (min_inliers is: " << params_.min_inliers << ")");

  if (static_cast<int>(inliers.size()) < params_.min_inliers)
    return false;

  // Save the loop closing image
  if (params_.save_graph_images)
  {
    // Build the keypoint vectors
    std::vector<cv::Point2f> keypoints_i = stereo_slam::Utils::stdMatrixToCvPoint2f(vert_i[0]->keypoints_.data());
    std::vector<cv::KeyPoint> cns_kp_i, cns_kp_j;
    for (uint k=0; k<keypoints_i.size(); k++)
    {
      cv::KeyPoint kp_i;
      kp_i.pt = keypoints_i[k];
      cns_kp_i.push_back(kp_i);
    }
    for (uint k=0; k<keypoints_j.size(); k++)
    {
      cv::KeyPoint kp_j;
      kp_j.pt = keypoints_j[k];
      cns_kp_j.push_back(kp_j);
    }
    std::vector<cv::DMatch> final_matches;
    for (uint k=0; k<inliers.size(); k++)
    {
      cv::DMatch match = matches[inliers[k]];
      final_matches.push_back(match);
    }

    // Read the vertices images
    std::string str_i = boost::lexical_cast<std::string>(v_i->id()+1);
    std::string str_j = boost::lexical_cast<std::string>(v_j->id()+1);
    cv::Mat img_i, img_j, canvas;
    img_i = cv::imread(params_.files_path + "img/" + str_i + ".jpg", CV_LOAD_IMAGE_COLOR);
    img_j = cv::imread(params_.files_path + "img/" + str_j + ".jpg", CV_LOAD_IMAGE_COLOR);

    cv::drawMatches(img_i, cns_kp_i, img_j, cns_kp_j, final_matches, canvas, 
        CV_RGB(0, 0, 255), CV_RGB(255, 0, 0), std::vector<char>());
    cv::imwrite(params_.files_path + "img/lc_" + str_i + "_" + str_j + ".jpg", canvas);
  }

  // Good! Loop closure, get the transformation matrix
  output = stereo_slam::Utils::buildTransformation(rvec, tvec);
  return true;
}