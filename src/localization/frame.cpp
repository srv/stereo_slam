#include <ros/ros.h>

#include "localization/frame.h"
#include "localization/ldb.h"

namespace slam
{

  Frame::Frame() : id_(-1), inliers_to_fixed_frame_(0) {}

  Frame::Frame(Mat l_img, Mat r_img, image_geometry::StereoCameraModel camera_model) : id_(-1), inliers_to_fixed_frame_(0)
  {
    l_img.copyTo(l_img_);
    r_img.copyTo(r_img_);

    // Convert images to grayscale
    Mat l_img_gray, r_img_gray;
    cvtColor(l_img, l_img_gray, CV_RGB2GRAY);
    cvtColor(r_img, r_img_gray, CV_RGB2GRAY);

    // Detect keypoints
    vector<KeyPoint> l_kp, r_kp;
    Ptr<FeatureDetector> cv_detector;
    cv_detector = FeatureDetector::create("ORB");
    cv_detector->detect(l_img_gray, l_kp);
    cv_detector->detect(r_img_gray, r_kp);

    // Extract descriptors
    Mat l_desc, r_desc;
    LDB extractor_;
    extractor_.compute(l_img_gray, l_kp, l_desc, 0);
    extractor_.compute(r_img_gray, r_kp, r_desc, 0);

    // Left/right matching
    Mat match_mask;
    const int knn = 2;
    const double ratio = 0.8;
    vector<DMatch> matches, matches_filtered;
    Ptr<DescriptorMatcher> descriptor_matcher;
    descriptor_matcher = DescriptorMatcher::create("BruteForce");
    vector<vector<DMatch> > knn_matches;
    descriptor_matcher->knnMatch(l_desc, r_desc, knn_matches, knn, match_mask);
    for (uint m=0; m<knn_matches.size(); m++)
    {
      if (knn_matches[m].size() < 2) continue;
      if (knn_matches[m][0].distance <= knn_matches[m][1].distance * ratio)
        matches.push_back(knn_matches[m][0]);
    }

    // Filter matches by epipolar
    for (size_t i=0; i<matches.size(); ++i)
    {
      if (abs(l_kp[matches[i].queryIdx].pt.y - r_kp[matches[i].trainIdx].pt.y) < 1.3)
        matches_filtered.push_back(matches[i]);
    }

    // Compute 3D points
    l_kp_.clear();
    r_kp_.clear();
    points_3d_.clear();
    l_desc_.release();
    r_desc_.release();
    for (size_t i=0; i<matches_filtered.size(); ++i)
    {
      Point3d world_point;
      int l_idx = matches_filtered[i].queryIdx;
      int r_idx = matches_filtered[i].trainIdx;

      Point2d l_point = l_kp[l_idx].pt;
      Point2d r_point = r_kp[r_idx].pt;

      double disparity = l_point.x - r_point.x;
      camera_model.projectDisparityTo3d(l_point, disparity, world_point);

      // Save
      if ( isfinite(world_point.x) && isfinite(world_point.y) && isfinite(world_point.z) && world_point.z > 0)
      {
        l_kp_.push_back(l_kp[l_idx]);
        r_kp_.push_back(r_kp[r_idx]);
        l_desc_.push_back(l_desc.row(l_idx));
        r_desc_.push_back(r_desc.row(r_idx));
        points_3d_.push_back(world_point);
      }
    }
  }

  Mat Frame::computeSift()
  {
    Mat sift;
    if (l_img_.cols == 0)
      return sift;

    Ptr<DescriptorExtractor> cv_extractor;
    cv_extractor = DescriptorExtractor::create("SIFT");
    cv_extractor->compute(l_img_, l_kp_, sift);

    return sift;
  }

} //namespace slam