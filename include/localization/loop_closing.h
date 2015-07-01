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

#include "constants.h"
#include "cluster.h"
#include "graph.h"
#include "hash.h"

using namespace std;
using namespace boost;
namespace fs  = filesystem;

namespace slam
{

class Graph;

class LoopClosing
{

public:

  /** \brief Class constructor
   */
  LoopClosing();

  /** \brief Set the graph object
   * \param graph
   */
  inline void setGraph(Graph *graph){graph_ = graph;}

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

  /** \brief Searches a loop closing between current cluster and its precedent neighbors
   */
  void searchInNeigborhood();

  /** \brief Searches a loop closing between current cluster and all other clusters using the hash
   */
  void searchByHash();

  /** \brief Get the best candidates to close a loop by hash
   * \param Cluster identifier
   * \param The list of best candidates
   */
  void getCandidates(int cluster_id, vector< pair<int,float> >& candidates);

  /** \brief Read cluster data from file
   * @return The cluster
   * \param Cluster file identifier
   */
  Cluster readCluster(string file);

private:

  Cluster c_cluster_; //!> Current cluster to be processed

  list<Cluster> cluster_queue_; //!> Clusters queue to be inserted into the graph

  mutex mutex_cluster_queue_; //!> Mutex for the insertion of new clusters

  Hash hash_; //!> Hash object

  vector< pair<int, vector<float> > > hash_table_;  //!> Hash table: stores a hash for every image. This is the unique variable that grows with the robot trajectory

  vector< pair<int, int > > lc_found_; //!> Stores all the loop closures found in order to do not repeat them

  string execution_dir_; //!> Execution directory where all image information will be stored

  Graph* graph_; //!> Graph pointer

  ros::Publisher pub_num_lc_; //!> Publishes the number of loop closings

  ros::Publisher pub_queue_; //!> Publishes the loop closing queue size

};

} // namespace

#endif // LOOP_CLOSING_H