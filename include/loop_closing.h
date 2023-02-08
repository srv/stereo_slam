/**
 * @file
 * @brief Loop closing class
 */

#ifndef LOOP_CLOSING_H
#define LOOP_CLOSING_H

#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "hash.h"
#include "constants.h"
#include "cluster.h"
#include "graph.h"

using namespace std;
using namespace boost;
namespace fs  = filesystem;

namespace slam
{

class Graph;

class LoopClosing
{

public:

  struct Params
  {
    int lc_neighbors;          //!> Number of neighbours to recover in order to increase the possibility of closing the loop.
    int lc_min_inliers;        //!> Minimum number of inliers to close a loop.
    int lc_discard_window;     //!> Window size of discarded vertices.
    double lc_epipolar_thresh; //!> Maximum reprojection error allowed.
    string working_directory;  //!> Directory where all output files will be stored.

    // Default settings
    Params ()
    {
      lc_neighbors       = 4;
      lc_min_inliers     = 40;
      lc_discard_window  = 15;
      lc_epipolar_thresh = 2.0;
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
  bool closeLoopWithCluster(Cluster candidate, string search_method);

  /** \brief Get the best candidates to close a loop by hash
   * \param Cluster identifier
   * \param The list of best candidates
   */
  void getCandidates(int cluster_id, vector< pair<int,float> >& candidates);

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
  void drawLoopClosure(vector<int> cand_kfs,
                       vector<int> cand_matchings,
                       vector<int> inliers,
                       vector<int> definitive_inliers_per_pair,
                       vector< vector<int> > definitive_cluster_pairs,
                       vector<cv::Point2f> matched_query_kp_l,
                       vector<cv::Point2f> matched_cand_kp_l);

private:

  Params params_; //!> Stores parameters.

  Cluster c_cluster_; //!> Current cluster to be processed

  list<Cluster> cluster_queue_; //!> Clusters queue to be inserted into the graph

  mutex mutex_cluster_queue_; //!> Mutex for the insertion of new clusters

  haloc::Hash hash_; //!> Hash object

  vector< pair<int, vector<float> > > hash_table_;  //!> Hash table: stores a hash for every image. This is the unique variable that grows with the robot trajectory

  vector< pair<int, int > > cluster_lc_found_; //!> Stores all the loop closures (between clusters) found in order to do not repeat them

  int num_loop_closures_; //!> Stores the number of loop closures

  string execution_dir_; //!> Execution directory where all image information will be stored

  string loop_closures_dir_; //!> Directory where images of loop closures will be stored

  Graph* graph_; //!> Graph pointer

  ros::Publisher pub_num_keyframes_; //!> Publishes the number of keyframes

  ros::Publisher pub_num_lc_; //!> Publishes the number of loop closings

  ros::Publisher pub_queue_; //!> Publishes the loop closing queue size

  ros::Publisher pub_matchings_num_; //!> Publishes the image with the loop closure matchings

  ros::Publisher pub_inliers_img_, pub_inliers_num_; //!> Publishes the image with the loop closure inliers

  ros::Publisher pub_matchings_percentage_; //!> Matching percentage

  image_geometry::PinholeCameraModel camera_model_; //!> Camera model (left)

};

} // namespace

#endif // LOOP_CLOSING_H