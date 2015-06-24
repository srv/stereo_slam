#include "localization/loop_closing.h"
#include "common/tools.h"

using namespace tools;

namespace slam
{

  LoopClosing::LoopClosing() {}

  void LoopClosing::run()
  {
    // Init
    execution_dir_ = WORKING_DIRECTORY + "loop_closing_" + lexical_cast<string>(time(0));
    if (fs::is_directory(execution_dir_))
      fs::remove_all(execution_dir_);
    fs::path dir(execution_dir_);
    if (!fs::create_directory(dir))
      ROS_ERROR("[Localization:] ERROR -> Impossible to create the loop_closing directory.");

    // Loop
    ros::Rate r(500);
    while(ros::ok())
    {
      if(checkNewFrameInQueue())
      {
        processNewFrame();

        searchByProximity();

        searchByHash();
      }
      r.sleep();
    }
  }

  void LoopClosing::addFrameToQueue(Frame frame)
  {
    mutex::scoped_lock lock(mutex_frame_queue_);
    frame_queue_.push_back(frame);
  }

  bool LoopClosing::checkNewFrameInQueue()
  {
    mutex::scoped_lock lock(mutex_frame_queue_);
    return(!frame_queue_.empty());
  }

  void LoopClosing::processNewFrame()
  {
    // Get the frame
    {
      mutex::scoped_lock lock(mutex_frame_queue_);
      c_frame_ = frame_queue_.front();
      frame_queue_.pop_front();
    }

    // Compute sift descriptors for this frame
    Mat desc = c_frame_.computeSift();

    // Initialize hash
    if (!hash_.isInitialized())
      hash_.init(desc);

    // Save hash to table
    hash_table_.push_back(make_pair(c_frame_.getId(), hash_.getHash(desc)));

    // Store
    FileStorage fs(execution_dir_+"/"+lexical_cast<string>(c_frame_.getId())+".yml", FileStorage::WRITE);
    write(fs, "id", c_frame_.getId());
    write(fs, "kp", c_frame_.getLeftKp());
    write(fs, "desc", c_frame_.getLeftDesc());
    write(fs, "threed", c_frame_.get3D());
    fs.release();
  }

  void LoopClosing::searchByProximity()
  {
    vector<int> neighbors = c_frame_.getGraphNeighbors();
    for (uint i=0; i<neighbors.size(); i++)
    {
      int inliers = 0;
      tf::Transform edge;
      bool valid = getLoopClosure(c_frame_.getId(), neighbors[i], edge, inliers);
      if (valid)
      {
        cout << "\033[1;32m[ INFO]: [Localization:] Node " << c_frame_.getId() << " closes loop with " << neighbors[i] << ".\033[0m\n";
        // TODO: add edge to the graph
        //addEdge(c_frame_.getId(), neighbors[i], edge);
      }
    }
  }

  void LoopClosing::searchByHash()
  {
    // TODO!
  }

  bool LoopClosing::getLoopClosure(int id_a, int id_b, tf::Transform& trans, int& inliers)
  {
    // Initialize outputs
    inliers = 0;
    trans.setIdentity();

    // Read frames a
    string file_a = execution_dir_+"/"+lexical_cast<string>(id_a)+".yml";
    string file_b = execution_dir_+"/"+lexical_cast<string>(id_b)+".yml";
    Frame frame_a = readFrame(file_a);
    Frame frame_b = readFrame(file_b);

    // Descriptors matching
    Mat match_mask;
    const int knn = 2;
    const double ratio = 0.8;
    vector<DMatch> matches;
    Ptr<DescriptorMatcher> descriptor_matcher;
    descriptor_matcher = DescriptorMatcher::create("BruteForce");
    vector<vector<DMatch> > knn_matches;
    descriptor_matcher->knnMatch(frame_b.getLeftDesc(), frame_a.getLeftDesc(), knn_matches, knn, match_mask);
    for (uint m=0; m<knn_matches.size(); m++)
    {
      if (knn_matches[m].size() < 2) continue;
      if (knn_matches[m][0].distance <= knn_matches[m][1].distance * ratio)
        matches.push_back(knn_matches[m][0]);
    }

    // Matches threshold
    if (matches.size() < MIN_INLIERS)
      return false;

    // Get the matched keypoints
    vector<KeyPoint> query_kp = frame_a.getLeftKp();
    vector<KeyPoint> candidate_kp = frame_b.getLeftKp();
    vector<Point3f> candidate_3d = frame_b.get3D();
    vector<Point2f> query_matched_kp;
    vector<Point2f> candidate_matched_kp;
    vector<Point3f> candidate_matched_3d_points;
    for(int i=0; i<matches.size(); i++)
    {
      query_matched_kp.push_back(query_kp[matches[i].trainIdx].pt);
      candidate_matched_kp.push_back(candidate_kp[matches[i].queryIdx].pt);
      candidate_matched_3d_points.push_back(candidate_3d[matches[i].queryIdx]);
    }

    // Estimate transform
    Mat rvec, tvec;
    vector<int> inliers_vector;
    solvePnPRansac(candidate_matched_3d_points, query_matched_kp,
                   graph_->getCameraMatrix(),
                   cv::Mat(), rvec, tvec, false,
                   100, 1.3,
                   MAX_INLIERS, inliers_vector);

    // Inliers threshold
    if (inliers_vector.size() < MIN_INLIERS)
      return false;

    trans = Tools::buildTransformation(rvec, tvec);
    inliers = inliers_vector.size();
    lc_found_.push_back(make_pair(id_a, id_b));
    return true;
  }

  Frame LoopClosing::readFrame(string file)
  {
    Frame frame;

    // Sanity check
    if ( !fs::exists(file) ) return frame;

    FileStorage fs;
    fs.open(file, FileStorage::READ);
    if (!fs.isOpened()) return frame;

    int id;
    vector<KeyPoint> kp;
    Mat desc;
    vector<Point3f> p3d;
    fs["id"] >> id;
    fs["desc"] >> desc;
    fs["threed"] >> p3d;
    FileNode kp_node = fs["kp"];
    read(kp_node, kp);
    fs.release();

    // Set the properties of the image
    frame.setId(id);
    frame.setLeftKp(kp);
    frame.setLeftDesc(desc);
    frame.set3D(p3d);

    return frame;
  }

  void LoopClosing::finalize()
  {
    // Remove the temporal directory
    if (fs::is_directory(execution_dir_))
      fs::remove_all(execution_dir_);
  }

} //namespace slam