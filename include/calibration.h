#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <ceres/ceres.h>
#include <tf/transform_datatypes.h>

#include "graph.h"

using namespace std;
using namespace boost;

namespace slam
{

class Graph;

class Calibration
{

public:

  /** \brief Hash class constructor
    */
  Calibration(Graph* graph);

  struct WorldPoint
  {
    /** \brief World point constructor.
     */
    WorldPoint(int id_a, int id_b, double disp_a, double disp_b, cv::Point2d uv_a, cv::Point2d uv_b);

    int frame_id_a;
    int frame_id_b;

    double disparity_a;
    double disparity_b;

    cv::Point2d point_uv_a;
    cv::Point2d point_uv_b;
  };

  struct CalibCostFunctor
  {

    CalibCostFunctor(tf::Transform tf_a, tf::Transform tf_b, cv::Point2d point_a, cv::Point2d point_b, double disp_a, double disp_b)
      : tfa_00_(tf_a.getBasis()[0][0]),
        tfa_01_(tf_a.getBasis()[0][1]),
        tfa_02_(tf_a.getBasis()[0][2]),
        tfa_03_(tf_a.getOrigin().x()),
        tfa_10_(tf_a.getBasis()[1][0]),
        tfa_11_(tf_a.getBasis()[1][1]),
        tfa_12_(tf_a.getBasis()[1][2]),
        tfa_13_(tf_a.getOrigin().y()),
        tfa_20_(tf_a.getBasis()[2][0]),
        tfa_21_(tf_a.getBasis()[2][1]),
        tfa_22_(tf_a.getBasis()[2][2]),
        tfa_23_(tf_a.getOrigin().z()),
        tfb_00_(tf_b.getBasis()[0][0]),
        tfb_01_(tf_b.getBasis()[0][1]),
        tfb_02_(tf_b.getBasis()[0][2]),
        tfb_03_(tf_b.getOrigin().x()),
        tfb_10_(tf_b.getBasis()[1][0]),
        tfb_11_(tf_b.getBasis()[1][1]),
        tfb_12_(tf_b.getBasis()[1][2]),
        tfb_13_(tf_b.getOrigin().y()),
        tfb_20_(tf_b.getBasis()[2][0]),
        tfb_21_(tf_b.getBasis()[2][1]),
        tfb_22_(tf_b.getBasis()[2][2]),
        tfb_23_(tf_b.getOrigin().z()),
        point_a_x_(point_a.x),
        point_a_y_(point_a.y),
        point_b_x_(point_b.x),
        point_b_y_(point_b.y),
        disp_a_(disp_a),
        disp_b_(disp_b)
    {}

    template <typename T>
    bool operator()(const T* const params, T* residuals) const
    {
      // Camera parameters
      T cx = params[0];
      T cy = params[1];
      T fx = params[2];

      // Compute the A world point
      T u_a = T(point_a_x_);
      T v_a = T(point_a_y_);
      T W_A = (1.0 / 0.121102) * T(disp_a_);
      T x_a = ( u_a - cx ) * (1.0 / W_A);
      T y_a = ( v_a - cy ) * (1.0 / W_A);
      T z_a = fx * (1.0 / W_A);
      T x_a_w = T(tfa_00_) * x_a + T(tfa_01_) * y_a + T(tfa_02_) * z_a + T(tfa_03_);
      T y_a_w = T(tfa_10_) * x_a + T(tfa_11_) * y_a + T(tfa_12_) * z_a + T(tfa_13_);
      T z_a_w = T(tfa_20_) * x_a + T(tfa_21_) * y_a + T(tfa_22_) * z_a + T(tfa_23_);

      // Compute the B world point
      T u_b = T(point_b_x_);
      T v_b = T(point_b_y_);
      T W_B = (1.0 / 0.121102) * T(disp_b_);
      T x_b = ( u_b - cx ) * (1.0 / W_B);
      T y_b = ( v_b - cy ) * (1.0 / W_B);
      T z_b = fx * (1.0 / W_B);
      T x_b_w = T(tfb_00_) * x_b + T(tfb_01_) * y_b + T(tfb_02_) * z_b + T(tfb_03_);
      T y_b_w = T(tfb_10_) * x_b + T(tfb_11_) * y_b + T(tfb_12_) * z_b + T(tfb_13_);
      T z_b_w = T(tfb_20_) * x_b + T(tfb_21_) * y_b + T(tfb_22_) * z_b + T(tfb_23_);

      residuals[0] = 1.0 - ((x_a_w*x_a_w) / (x_b_w*x_b_w));
      residuals[1] = 1.0 - ((y_a_w*y_a_w) / (y_b_w*y_b_w));
      residuals[2] = 1.0 - ((z_a_w*z_a_w) / (z_b_w*z_b_w));

      return true;
    }

    double tfa_00_, tfa_01_, tfa_02_, tfa_03_, tfa_10_, tfa_11_, tfa_12_, tfa_13_, tfa_20_, tfa_21_, tfa_22_, tfa_23_;
    double tfb_00_, tfb_01_, tfb_02_, tfb_03_, tfb_10_, tfb_11_, tfb_12_, tfb_13_, tfb_20_, tfb_21_, tfb_22_, tfb_23_;
    double point_a_x_, point_a_y_;
    double point_b_x_, point_b_y_;
    double disp_a_, disp_b_;
  };

  void setCameraParameters(double cx, double cy, double fx);

  /** \brief Adds a set of world points to the calibration process
   * \param vector of world points
   */
  void update(vector<WorldPoint> world_points);

  /** \brief Runs the calibration
   */
  void run();

  /** \brief Returns the camera parameters
   * @return The vector of camera parameters [cx, cy, fx]
   */
  vector<double> getCameraParams();

private:

  vector<WorldPoint> all_points_; //!> Vector containing all world points

  mutex mutex_points_; //!> Mutex for the vector of points manipulation

  double* camera_params_; //!> Camera parameters vector [cx, cy, fx]

  Graph* graph_; //!> Graph

};

} // namespace

#endif // CALIBRATION_H