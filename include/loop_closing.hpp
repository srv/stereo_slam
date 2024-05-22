/**
 * @file
 * @brief Loop closing class
 */

#ifndef LOOP_CLOSING_H
#define LOOP_CLOSING_H

#include <numeric>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <image_geometry/pinhole_camera_model.h>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>

#include "hash.hpp"
#include "cluster.hpp"
#include "graph.hpp"
#include "stereo_slam/TimeLoopClosing.h"
#include "stereo_slam/SubTimeLoopClosing.h"

namespace slam
{

class Graph;

class LoopClosing
{

public:

  struct Params
  {
    int lc_neighbors;                   //!> Number of neighbours to recover in order to increase the possibility of closing the loop.
    int lc_discard_window;              //!> Window size of discarded vertices.
    double lc_epipolar_thresh;          //!> Maximum reprojection error allowed.
    long unsigned int lc_min_inliers;   //!> Minimum number of inliers to close a loop.
    int ransac_iterations;              //!> Number of RANSAC iterations for the solvePnPRansac
    std::string working_directory;      //!> Directory where all output files will be stored.

    // Default settings
    Params ()
    {
      lc_neighbors       = 4;
      lc_min_inliers     = 40;
      lc_discard_window  = 15;
      lc_epipolar_thresh = 2.0;
      ransac_iterations  = 100;
      working_directory  = "";
    }
  };

  /** \brief Class constructor
   */
  LoopClosing();

  /** \brief Set the graph object
   * \param graph
   */
  inline void setGraph(Graph *graph){graph_ = graph;}

  /** \brief Set class params
   *  \param the parameters struct
   */
  inline void setParams(const Params& params){params_ = params;}

  /** \brief Get class params
   */
  inline Params getParams() const {return params_;}

  /** \brief Get cluster counter
   */
  inline int getClusterNum() const {return cluster_id_ + 1;}

  /** \brief Starts graph
   */
  void run();

  /** \brief Add a cluster to the queue of clusters
   * \param The cluster to be inserted
   */
  void addClusterToQueue(Cluster cluster);

  /** \brief Finalizes the loop closing class
   */
  void finalize();

protected:

  /** \brief Check if there are clusters in the queue
   * @return true if clusters queue is not empty.
   */
  bool checkNewClusterInQueue();

  /** \brief Processes the new cluster
   */
  void processNewCluster();

  /** \brief Searches a loop closing between current cluster and its closest neighbors
   */
  void searchByProximity();

  /** \brief Searches a loop closing between current cluster and all other clusters using the hash
   */
  void searchByHash();

  /** \brief Tries to close a loop between two clusters
   * @return true if loop closing
   * \param Candidate cluster
   * \param Type of search (proximity or hash)
   */
  bool closeLoopWithCluster(Cluster candidate, std::string search_method);

  void publishSubTimeloopClosing();

  /** \brief Get the best candidates to close a loop by hash
   * \param Cluster identifier
   * \param The list of best candidates
   */
  void getCandidates(int cluster_id, std::vector< std::pair<int,float> >& candidates);

  /** \brief Read cluster data from file
   * @return The cluster
   * \param Cluster identifier
   */
  Cluster readCluster(int id);

  /** \brief Draw and publish a loop closure image with all the correspondences between current keyframe and all the loop closing keyframes
   * \param The loop closing keyframe identifiers
   * \param The loop closing cluster identifiers
   * \param The inlier indices
   * \param The number of inliers for every loop closure cluster pair
   * \param The cluster pairs vector
   * \param All matched keypoints of the current keyframe
   * \param All matched keypoints of the candidate keyframes
   */
  void drawLoopClosure(std::vector<int> cand_kfs,
                       std::vector<int> cand_matchings,
                       std::vector<int> inliers,
                       std::vector<int> definitive_inliers_per_pair,
                       std::vector< std::vector<int> > definitive_cluster_pairs,
                       std::vector<cv::Point2f> matched_query_kp_l,
                       std::vector<cv::Point2f> matched_cand_kp_l);

private:

  bool first_time_ = true; 

  int cluster_id_; //!> Processed cluster counter

  Params params_; //!> Stores parameters.

  Cluster c_cluster_; //!> Current cluster to be processed

  std::list<Cluster> cluster_queue_; //!> Clusters queue to be inserted into the graph

  boost::mutex mutex_cluster_queue_; //!> Mutex for the insertion of new clusters

  haloc::Hash hash_; //!> Hash object

  std::vector< std::pair<int, std::vector<float> > > hash_table_;  //!> Hash table: stores a hash for every image. This is the unique variable that grows with the robot trajectory

  std::vector< std::pair<int, int > > cluster_lc_found_; //!> Stores all the loop closures (between clusters) found in order to do not repeat them

  int num_loop_closures_; //!> Stores the number of loop closures

  std::string execution_dir_; //!> Execution directory where all image information will be stored

  std::string loop_closures_dir_; //!> Directory where images of loop closures will be stored

  Graph* graph_; //!> Graph pointer

  ros::Publisher pub_queue_; //!> Publishes the loop closing queue size

  ros::Publisher pub_num_lc_; //!> Publishes the number of loop closings

  ros::Publisher pub_num_clusters_; //!> Publishes the number of clusters

  ros::Publisher pub_matchings_num_; //!> Publishes the image with the loop closure matchings

  ros::Publisher pub_matchings_percentage_; //!> Matching percentage

  image_geometry::PinholeCameraModel camera_model_; //!> Camera model (left)

  ros::Publisher pub_inliers_img_, pub_inliers_num_; //!> Publishes the image with the loop closure inliers

  stereo_slam::TimeLoopClosing time_loop_closing_msg_; //! Message to publish time metrics

  stereo_slam::SubTimeLoopClosing sub_time_loop_closing_msg_; //! Message to publish specific time metrics

  ros::Publisher pub_time_loop_closing_, pub_sub_time_loop_closing_; //!> Time loop closing thread publisher

};

} // namespace

#endif // LOOP_CLOSING_HPP