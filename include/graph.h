/**
 * @file
 * @brief Graph class
 */

#ifndef GRAPH_H
#define GRAPH_H

#include <memory>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/lexical_cast.hpp>

#include "frame.h"
#include "loop_closing.h"
#include "stereo_slam/GraphPoses.h"
#include "stereo_slam/TimeGraph.h"

namespace slam
{

class LoopClosing;

class Graph
{

public:

  struct Params
  {
    std::string map_frame_id;      //!> Frame of the slam output
    std::string working_directory; //!> Directory where all output files will be stored.

    // Default settings
    Params ()
    {
      map_frame_id = "map";
      working_directory = "";
    }
  };

  struct Edge
  {
    int vertice_a;
    int vertice_b;
    int inliers;

    // Default settings
    Edge (int a, int b, int inlier) {
      vertice_a  = a;
      vertice_b  = b;
      inliers = inlier;
    }
  };

	/** \brief Class constructor
   * \param Loop closing object pointer
   */
  Graph(LoopClosing* loop_closing);

  /** \brief Initialize the graph
   */
  void init();

  /** \brief Starts graph
   */
  void run();

  /** \brief Add a frame to the queue of frames to be inserted into the graph as vertices
   * \param The frame to be inserted
   */
  void addFrameToQueue(Frame frame);

  /** \brief Add an edge to the graph
   * \param Index of vertex 1
   * \param Index of vertex 2
   * \param Transformation between vertices
   * \param Sigma information
   * \param Inliers
   */
  void addEdge(int i, int j, tf::Transform edge, cv::Mat sigma, int inliers);

  /** \brief Optimize the graph
   */
  void update();

  /** \brief Get the closest neighbors by distance
   * \param The vertex id to retrieve its neighbors
   * \param The vertex where discard window will be centered.
   * \param Window size of discarded vertices.
   * \param Number of neighbors to be retrieved.
   * \param Will contain the list of best neighbors by distance.
   */
  void findClosestVertices(int vertex_id, int window_center, int window, int best_n, std::vector<int> &neighbors);

  /** \brief Retrieve the list of the vertices of a corresponding frame
   * \param The frame id
   * \param Will contain the list of vertices for this frame.
   */
  void getFrameVertices(int frame_id, std::vector<int> &vertices);

  /** \brief Get the frame id of some specific vertex
   * @return the frame id
   * \param vertex id
   */
  int getVertexFrameId(int id);

  /** \brief Get the last vertex frame id
   * @return the frame id
   */
  int getLastVertexFrameId();

  /** \brief Get the graph vertex pose
   * @return graph vertex pose
   * \param vertex id
   * \param set to true to lock the graph
   */
  tf::Transform getVertexPose(int vertex_id, bool lock = true);

  /** \brief Get frame pose
   * @return true if frame pose can be extracted, false otherwise
   * \param frame id
   * \param output graph frame pose
   */
  bool getFramePose(int frame_id, tf::Transform& frame_pose);

  /** \brief Get the graph vertex pose relative to camera
   * @return graph vertex pose relative to camera
   * \param vertex id
   */
  tf::Transform getVertexPoseRelativeToCamera(int id);

  /** \brief Get the camera pose of some specific vertex
   * @return graph vertex camera pose
   * \param vertex id
   * \param true to lock the graph
   */
  tf::Transform getVertexCameraPose(int id, bool lock = true);

  /** \brief Save the graph to file
   */
  void saveGraph();

  /** \brief Set the transformation between camera and robot frame 
   * \param the transform
   */
  inline void setCamera2Robot(const tf::Transform& camera2robot){camera2robot_ = camera2robot;}

  /** \brief Set the odom frame
   * \param the odom frame id
   */
  inline void setOdomFrame(const std::string& odom_frame_id){odom_frame_id_ = odom_frame_id;}

  /** \brief Set camera matrix
   * \param camera matrix
   */
  inline void setCameraMatrix(const cv::Mat& camera_matrix){camera_matrix_ = camera_matrix;}

  /** \brief Get camera matrix
   */
  inline cv::Mat getCameraMatrix() const {return camera_matrix_;}

  /** \brief Set camera model
   * \param camera matrix
   */
  inline void setCameraModel(const image_geometry::PinholeCameraModel& camera_model){camera_model_ = camera_model;}

  /** \brief Get camera model
   */
  inline image_geometry::PinholeCameraModel getCameraModel() const {return camera_model_;}

  /** \brief Get frame counter
   */
  inline int getFrameNum() const {return frame_id_ + 1;}

  /** \brief Set class params
   *  \param the parameters struct
   */
  inline void setParams(const Params& params){params_ = params;}

  /** \brief Get class params
   */
  inline Params getParams() const {return params_;}

protected:

  /** \brief Correct a cluster pose with the information of the updated graph
   * @return the corrected pose
   * \param The pose to be corrected
   */
  tf::Transform correctClusterPose(tf::Transform initial_pose);

  /** \brief Return all possible combinations of 2 elements of the input vector
   * @return the list of combinations
   * \param Input vector with all values
   */
  std::vector< std::vector<int> > createComb(std::vector<int> cluster_ids);

  /** \brief Check if there are frames in the queue to be inserted into the graph
   * @return true if frames queue is not empty.
   */
  bool checkNewFrameInQueue();

  /** \brief Converts the frame to a graph vertex and adds it to the graph
   */
  void processNewFrame();

  /** \brief Add a vertex to the graph
   * @return the vertex id
   * \param Vertex pose
   */
  int addVertex(tf::Transform pose);

  /** \brief Save the frame to the default location
   * \param the frame to be drawn
   */
  void saveFrame(Frame frame);

  /** \brief Publishes the graph camera pose
   * \param Camera pose
   */
  void publishUpdatedPose(tf::Transform camera_pose);

  /** \brief Publishes all the graph
   */
  void publishGraph();

private:

  Params params_; //!> Stores parameters.

  g2o::SparseOptimizer graph_optimizer_; //!> G2O graph optimizer

  std::list<Frame> frame_queue_; //!> Frames queue to be inserted into the graph

  int frame_id_; //!> Processed frames counter

  std::vector< std::pair< int,int > > cluster_frame_relation_; //!> Stores the cluster/frame relation (cluster_id, frame_id)

  std::vector<tf::Transform> local_cluster_poses_; //!> Stores the cluster poses relative to camera frame

  std::vector<tf::Transform> initial_cluster_pose_history_; //!> Stores the initial cluster poses, before graph update.

  std::vector<double> frame_stamps_; //> Stores the frame timestamps

  boost::mutex mutex_graph_; //!> Mutex for the graph manipulation

  boost::mutex mutex_frame_queue_; //!> Mutex for the insertion of new frames into the graph

  tf::Transform camera2robot_; //!> Transformation between camera and robot frame

  std::string odom_frame_id_; //!> Odom frame 

  LoopClosing* loop_closing_; //!> Loop closing

  cv::Mat camera_matrix_; //!> The camera matrix

  image_geometry::PinholeCameraModel camera_model_; //!> Pinhole left camera model

  ros::Publisher pub_graph_; //!> Graph publisher

  ros::Publisher pub_time_graph_; //!> Time graph thread publisher

  ros::Publisher pub_robot_pose_; //!> Updated robot pose publisher
  
  ros::Publisher pub_camera_pose_; //!> Updated camera pose publisher

  ros::Publisher pub_num_keyframes_; //!> Publishes the number of keyframes

  stereo_slam::TimeGraph time_graph_msg_; //! Message to publish time metrics 

  std::vector<Edge> edges_information_; // Edges information
};

} // namespace

#endif // GRAPH_H
