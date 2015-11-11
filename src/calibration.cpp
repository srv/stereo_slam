#include <ros/ros.h>

#include "calibration.h"

namespace slam
{

  Calibration::WorldPoint::WorldPoint(int id_a, int id_b, double disp_a, double disp_b, cv::Point2d uv_a, cv::Point2d uv_b) :
    frame_id_a(id_a), frame_id_b(id_b), disparity_a(disp_a), disparity_b(disp_b), point_uv_a(uv_a), point_uv_b(uv_b)
  {}

  Calibration::Calibration(Graph *graph) : graph_(graph)
  {
    // Init
    all_points_.clear();
    camera_params_ = new double[6];
  }

  void Calibration::setCameraParameters(double Tx, double cx, double cy, double fx)
  {
    camera_params_[0] = Tx;
    camera_params_[1] = cx;
    camera_params_[2] = cy;
    camera_params_[3] = fx;
  }

  void Calibration::update(vector<WorldPoint> world_points)
  {
    mutex::scoped_lock lock(mutex_points_);

    if (world_points.size() > 0)
      all_points_.insert(all_points_.end(), world_points.begin(), world_points.end());
  }

  vector<double> Calibration::getCameraParams()
  {
    vector<double> out;
    out.push_back(camera_params_[0]);
    out.push_back(camera_params_[1]);
    out.push_back(camera_params_[2]);
    out.push_back(camera_params_[3]);
    return out;
  }

  void Calibration::run()
  {
    // Ceres problem
    ceres::Problem problem;

    // Get the points
    vector<WorldPoint> all_points;
    {
      mutex::scoped_lock lock(mutex_points_);
      all_points = all_points_;
    }

    cout << "[Localization:] Optimizing camera parameters with " << all_points_.size() << " points." << endl;
    cout << "[Localization:] Initial parameters (Tx, cx, cy, fx): " << camera_params_[0] << ", " << camera_params_[1] << ", " << camera_params_[2] << ", " << camera_params_[3] << endl;

    for (uint i=0; i<all_points.size(); i++)
    {
      WorldPoint p = all_points[i];

      tf::Transform tf_a, tf_b;
      bool valid_a = graph_->getFramePose(p.frame_id_a, tf_a);
      bool valid_b = graph_->getFramePose(p.frame_id_b, tf_b);
      if (!valid_a || ! valid_b)
        continue;

      cv::Point2d p_a = p.point_uv_a;
      cv::Point2d p_b = p.point_uv_b;

      double disp_a = p.disparity_a;
      double disp_b = p.disparity_b;

      ceres::CostFunction* cost_function =
             new ceres::AutoDiffCostFunction<CalibCostFunctor, 3, 4>(
                 new CalibCostFunctor(tf_a, tf_b, p_a, p_b, disp_a, disp_b));

      problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.2), camera_params_);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.max_num_iterations = 1000;
    options.minimizer_progress_to_stdout = false;
    options.max_solver_time_in_seconds = 600;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    cout << "[Localization:] Final parameters (Tx, cx, cy, fx): " << camera_params_[0] << ", " << camera_params_[1] << ", " << camera_params_[2] << ", " << camera_params_[3] << endl;
    cout << endl << endl << endl;

    //cout << summary.FullReport() << "\n";
  }

} //namespace slam