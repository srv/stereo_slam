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

    for (unsigned int j=i+2; j<graph_size; j++)
    {
      // Extract the pose of vertex j and compare
      g2o::VertexSE3* v_j = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[j]);
      tf::Transform pose_j = stereo_slam::Utils::getVertexPose(v_j);

      // Check if this have been discarted previously
      bool false_cand = stereo_slam::Utils::searchFalseCandidates(false_candidates_, v_i->id(), v_j->id());

      // Proceed if this combination is possible
      if (!false_cand)
      {
        // Check if there is an edge that currently join both vertices
        bool edge_found = false;
        for (g2o::OptimizableGraph::EdgeSet::iterator it=graph_optimizer_.edges().begin();
         it!=graph_optimizer_.edges().end(); it++)
        {
          g2o::EdgeSE3* e = dynamic_cast<g2o::EdgeSE3*> (*it);
          if (e)
          {
            if (  (e->vertices()[0]->id() == v_i->id() && e->vertices()[1]->id() == v_j->id()) ||
                  (e->vertices()[0]->id() == v_j->id() && e->vertices()[1]->id() == v_i->id()) )
            {
              edge_found = true;
              break;
            }
          }
        }        

        // If no edges found connecting this vertices, try to find loop closures
        bool is_false = true;
        double pose_diff = stereo_slam::Utils::poseDiff(pose_i, pose_j);
        if (!edge_found && (pose_diff < params_.max_candidate_threshold)
                        && (pose_diff > params_.min_candidate_threshold))
        {
          // Get the data of both vertices from database
          std::string where_i = "(id = " + boost::lexical_cast<std::string>(v_i->id() + 1) + ")";
          std::string where_j = "(id = " + boost::lexical_cast<std::string>(v_j->id() + 1) + ")";
          std::vector< boost::shared_ptr<stereo_slam::GraphDB> > vert_i;
          std::vector< boost::shared_ptr<stereo_slam::GraphDB> > vert_j;
          pg_db_ptr_thread_2_->getList(vert_i, where_i);
          pg_db_ptr_thread_2_->getList(vert_j, where_j);
          if (vert_i.size() == 1 && vert_j.size() == 1)
          {
            cv::Mat desc_i = cv::Mat_<std::vector<float> >();
            cv::Mat desc_j = cv::Mat_<std::vector<float> >();
            desc_i = stereo_slam::Utils::stdMatrixToCvMat(vert_i[0]->descriptors_.data());
            desc_j = stereo_slam::Utils::stdMatrixToCvMat(vert_j[0]->descriptors_.data());

            // Check for enought descriptors
            if (desc_i.rows > params_.matches_threshold && desc_j.rows > params_.matches_threshold)
            {
              // Compute matchings
              std::vector<cv::DMatch> matches;
              stereo_slam::Utils::thresholdMatching(desc_i, desc_j, matches, params_.descriptor_threshold);

              if (params_.stereo_vision_verbose)
                ROS_INFO_STREAM("[StereoSlam:] Found " << matches.size() <<
                   " matches between vertices " << v_i->id() << " and " << v_j->id() <<
                   " (matches_threshold is: " << params_.matches_threshold << ")");

              if ((int)matches.size() > params_.matches_threshold)
              {
                // Extract keypoints and 3d points of vertex i and j
                std::vector<cv::Point2f> keypoints_j;
                std::vector<cv::Point3f> points3d_i;
                keypoints_j = stereo_slam::Utils::stdMatrixToCvPoint2f(vert_j[0]->keypoints_.data());
                points3d_i = stereo_slam::Utils::stdMatrixToCvPoint3f(vert_i[0]->points3d_.data());
                std::vector<cv::Point2f> matched_keypoints;
                std::vector<cv::Point3f> matched_3d_points;
                for (size_t i = 0; i < matches.size(); ++i)
                {
                  int index_left = matches[i].queryIdx;
                  int index_right = matches[i].trainIdx;;
                  matched_3d_points.push_back(points3d_i[index_left]);
                  matched_keypoints.push_back(keypoints_j[index_right]);
                }
                
                // Compute the transformation between the vertices
                cv::Mat rvec, tvec;
                std::vector<int> inliers;
                cv::solvePnPRansac(matched_3d_points, matched_keypoints, camera_matrix_, 
                                   cv::Mat(), rvec, tvec, false, 
                                   params_.max_solvepnp_iter, params_.allowed_reprojection_err, 
                                   params_.max_inliers, inliers);

                if (params_.stereo_vision_verbose)
                  ROS_INFO_STREAM("[StereoSlam:] Found " << inliers.size() <<
                   " inliers between vertices " << v_i->id() << " and " << v_j->id() <<
                   " (min_inliers is: " << params_.min_inliers << ")");

                if (static_cast<int>(inliers.size()) >= params_.min_inliers)
                {
                  // Good! Loop closure, get the transformation matrix
                  tf::Transform cl_edge = stereo_slam::Utils::buildTransformation(rvec, tvec);

                  // To prevent for possible errors, compare previous transform with the new edge found
                  Eigen::Isometry3d t = v_i->estimate().inverse() * v_j->estimate();
                  tf::Transform cl_edge_prev = stereo_slam::Utils::eigenToTf(t);

                  // Check edge geometry
                  if (stereo_slam::Utils::poseDiff(cl_edge, cl_edge_prev) < params_.max_edge_err)
                  {
                    // Add the new edge to graph
                    g2o::EdgeSE3* e = new g2o::EdgeSE3();
                    Eigen::Isometry3d t = stereo_slam::Utils::tfToEigen(cl_edge);
                    e->setVertex(0, v_j);
                    e->setVertex(1, v_i);
                    e->setMeasurement(t);
                    graph_optimizer_.addEdge(e);
                    edge_added = true;
                    is_false = false;

                    ROS_INFO_STREAM("[StereoSlam:] Loop closed between vertices " << v_i->id() << " and " << v_j->id());
                  }
                }
              }
            }
          }
        }

        // Bad candidate, save to prevent future processes
        if (is_false)
        {
          cv::Point2i vert(v_i->id(), v_j->id());
          false_candidates_.push_back(vert);
        }
      }
    }
  }

  return edge_added;
}