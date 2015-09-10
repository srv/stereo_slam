#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

using namespace cv;
using namespace std;


vector<DMatch> matching(Mat desc_1, Mat desc_2, double th)
{
  vector<DMatch> matches;
  const int knn = 2;
  Mat match_mask;
  Ptr<DescriptorMatcher> descriptor_matcher;

  if (desc_1.type() == CV_8U)
    descriptor_matcher = DescriptorMatcher::create("BruteForce-Hamming");
  else
    descriptor_matcher = DescriptorMatcher::create("BruteForce");

  vector<vector<DMatch> > knn_matches;
  descriptor_matcher->knnMatch(desc_1, desc_2, knn_matches, knn, match_mask);

  for (unsigned m = 0; m < knn_matches.size(); m++)
  {
    if (knn_matches[m].size() < 2) continue;
    if (knn_matches[m][0].distance <= knn_matches[m][1].distance * th)
      matches.push_back(knn_matches[m][0]);
  }

  return matches;
}

Mat draw(Mat img, vector<DMatch> matches, vector<KeyPoint> kp)
{
  Mat paint;
  img.copyTo(paint);

  vector<int> matches_int;
  for (uint i=0; i<matches.size(); i++)
    matches_int.push_back(matches[i].queryIdx);

  const float r = 5;
  for(uint i=0; i<kp.size(); i++)
  {
    cv::Scalar color;
    if (find(matches_int.begin(), matches_int.end(), i) != matches_int.end())
    {
      color = cv::Scalar(29,255,45);
      Point2f pt1, pt2;
      pt1.x = kp[i].pt.x-r;
      pt1.y = kp[i].pt.y-r;
      pt2.x = kp[i].pt.x+r;
      pt2.y = kp[i].pt.y+r;

      // rectangle(paint, pt1, pt2, color, 2);
      circle(paint, kp[i].pt, 5, color, -1);
    }
  }
  return paint;
}


int main(int argc, char** argv)
{
  Mat img1 = imread( "/home/plnegre/workspace/ros/turbot/src/stereo_slam/test/1.jpg", 1 );
  Mat img2 = imread( "/home/plnegre/workspace/ros/turbot/src/stereo_slam/test/2.jpg", 1 );

  initModule_nonfree();

  // Detect keypoints
  vector<KeyPoint> kp_orb_1, kp_sift_1, kp_surf_1, kp_fast_1, kp_orb_2, kp_sift_2, kp_surf_2, kp_fast_2;
  Mat desc_orb_1, desc_sift_1, desc_surf_1, desc_fast_1, desc_orb_2, desc_sift_2, desc_surf_2, desc_fast_2;
  Ptr<FeatureDetector> cv_detector;
  Ptr<DescriptorExtractor> cv_extractor;

  // ORB
  cv_detector = FeatureDetector::create("ORB");
  cv_extractor = DescriptorExtractor::create("ORB");
  cv_detector->detect(img1, kp_orb_1);
  cv_detector->detect(img2, kp_orb_2);
  cv_extractor->compute(img1, kp_orb_1, desc_orb_1);
  cv_extractor->compute(img2, kp_orb_2, desc_orb_2);
  vector<DMatch> matches_orb = matching(desc_orb_1, desc_orb_2, 0.8);
  cout << "ORB" << endl;

  // SIFT
  cv_detector = FeatureDetector::create("SIFT");
  cv_extractor = DescriptorExtractor::create("SIFT");
  cv_detector->detect(img1, kp_sift_1);
  cv_detector->detect(img2, kp_sift_2);
  cv_extractor->compute(img1, kp_sift_1, desc_sift_1);
  cv_extractor->compute(img2, kp_sift_2, desc_sift_2);
  vector<DMatch> matches_sift = matching(desc_sift_1, desc_sift_2, 0.6);
  cout << "SIFT" << endl;

  // SURF
  cv_detector = FeatureDetector::create("SURF");
  cv_extractor = DescriptorExtractor::create("SURF");
  cv_detector->detect(img1, kp_surf_1);
  cv_detector->detect(img2, kp_surf_2);
  cv_extractor->compute(img1, kp_surf_1, desc_surf_1);
  cv_extractor->compute(img2, kp_surf_2, desc_surf_2);
  vector<DMatch> matches_surf = matching(desc_surf_1, desc_surf_2, 0.6);
  cout << "SURF" << endl;

  // FAST
  cv_detector = FeatureDetector::create("FAST");
  cv_extractor = DescriptorExtractor::create("SURF");
  cv_detector->detect(img1, kp_fast_1);
  cv_detector->detect(img2, kp_fast_2);
  cv_extractor->compute(img1, kp_fast_1, desc_fast_1);
  cv_extractor->compute(img2, kp_fast_2, desc_fast_2);
  vector<DMatch> matches_fast = matching(desc_fast_1, desc_fast_2, 0.6);
  cout << "FAST" << endl;

  Mat orb = draw(img1, matches_orb, kp_orb_1);
  Mat sift = draw(img1, matches_sift, kp_sift_1);
  Mat surf = draw(img1, matches_surf, kp_surf_1);
  Mat fast = draw(img1, matches_fast, kp_fast_1);

  imwrite( "/home/plnegre/workspace/ros/turbot/src/stereo_slam/test/orb.png", orb );
  imwrite( "/home/plnegre/workspace/ros/turbot/src/stereo_slam/test/sift.png", sift );
  imwrite( "/home/plnegre/workspace/ros/turbot/src/stereo_slam/test/surf.png", surf );
  imwrite( "/home/plnegre/workspace/ros/turbot/src/stereo_slam/test/fast.png", fast );

  namedWindow("orb", CV_WINDOW_AUTOSIZE);
  imshow("orb", orb);
  namedWindow("sift", CV_WINDOW_AUTOSIZE);
  imshow("sift", sift);
  namedWindow("surf", CV_WINDOW_AUTOSIZE);
  imshow("surf", surf);
  namedWindow("fast", CV_WINDOW_AUTOSIZE);
  imshow("fast", fast);

  waitKey(0);
  return 0;
}

