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
#include "frame.h"
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

  /** \brief Add a frame to the queue of frames to be inserted into the graph as vertices
   * \param The frame to be inserted
   */
  void addFrameToQueue(Frame frame);

  /** \brief Finalizes the loop closing class
   */
  void finalize();

protected:

  /** \brief Check if there are frames in the queue to be inserted into the graph
   * @return true if frames queue is not empty.
   */
  bool checkNewFrameInQueue();

  /** \brief Converts the frame to a graph vertex and adds it to the graph
   */
  void processNewFrame();

  /** \brief Searches a loop closing between current processed frame and its closest neighbors
   */
  void searchByProximity();

  /** \brief Searches a loop closing between current processed frame and all other frames using the hash
   */
  void searchByHash();

  /** \brief Compute the loop closure (if any) between A -> B.
   * @return true if valid loop closure, false otherwise.
   * \param reference frame id.
   * \param candidate frame id.
   * \param Return the transform between nodes if loop closure is valid.
   * \param The number of inliers
   */
  bool getLoopClosure(int id_a,
                      int id_b,
                      tf::Transform& trans,
                      int& inliers);

  /** \brief Read frame data from file
   * @return The frame
   * \param Frame file identifier
   */
  Frame readFrame(string file);

  /** \brief Get the best n_candidates to close loop with the image specified by id.
   * @return a hash matching vector containing the best image candidates and its likelihood.
   * \param image id.
   */
  void getCandidates(int frame_id, vector< pair<int,float> >& candidates);

  /** \brief Retrieve the best n matchings give a certain image id.
   * \param Query image id.
   * \param Number of best matches to retrieve.
   * \param Stores best n matchings in a vector of pairs <image_id, distance>.
   */
   void getBestMatchings(int frame_id, vector< pair<int,float> > &best_matchings);

   /** \brief Compute the likelihood vectors given a certain hash matching set.
    * \param Hash matching vector in the format <image_id, distance>.
    * \param Stores the likelihood in the format <image_id, probability>.
    */
   void buildLikelihoodVector(vector< pair<int,float> > hash_matchings,
                              vector< pair<int,float> > &likelihood);


   /** \brief Given a vector of matches and the current and previous likelihood vectors, returns the
    * likelihood for these matches.
    * \param Hash matching vector in the format <image_id, distance>.
    * \param Stores the likelihood for the given matching vectors
    * \param Current vector of likelihood in the format <image_id, probability>.
    * \param Previous vector of likelihood in the format <image_id, probability>.
    */
   void getMatchingsLikelihood(vector< pair<int,float> > matchings,
                               vector<float> &matchings_likelihood,
                               vector< pair<int,float> > cur_likelihood,
                               vector< pair<int,float> > prev_likelihood);

   /** \brief Group the matchings by images with similar id
    * \param Hash matching vector in the format <image_id, distance>.
    * \param Stores groups of images
    */
   void groupSimilarImages(vector< pair<int,float> > matchings,
                           vector< vector<int> > &groups);
private:

  Frame c_frame_; //!> Current frame to be processed

  list<Frame> frame_queue_; //!> Frames queue to be inserted into the graph

  mutex mutex_frame_queue_; //!> Mutex for the insertion of new frames into the graph

  Hash hash_; //!> Hash object

  vector< pair<int, vector<float> > > hash_table_;  //!> Hash table: stores a hash for every image. This is the unique variable that grows with the robot trajectory

  vector< pair<int, int > > lc_found_; //!> Stores all the loop closures found in order to do not repeat them

  vector< pair<int,float> > prev_likelihood_; //!> Stores the previous likelihood vector

  string execution_dir_; //!> Execution directory where all image information will be stored

  Graph* graph_; //!> Graph pointer

};

} // namespace

#endif // LOOP_CLOSING_H