#include <std_msgs/Int32.h>

#include <numeric>

#include <boost/math/distributions.hpp>

#include "localization/loop_closing.h"
#include "common/tools.h"

using namespace tools;

namespace slam
{

  LoopClosing::LoopClosing()
  {
    ros::NodeHandle nhp("~");
    pub_num_lc_ = nhp.advertise<std_msgs::Int32>("loop_closings", 2, true);
    pub_queue_ = nhp.advertise<std_msgs::Int32>("loop_closing_queue", 2, true);
  }

  void LoopClosing::run()
  {
    // Init
    execution_dir_ = WORKING_DIRECTORY + "loop_closing";
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

        searchInNeighbors();

        searchByProximity();

        searchByHash();
      }

      if (pub_num_lc_.getNumSubscribers() > 0)
      {
        std_msgs::Int32 msg;
        msg.data = lc_found_.size();
        pub_num_lc_.publish(msg);
      }

      if (pub_queue_.getNumSubscribers() > 0)
      {
        mutex::scoped_lock lock(mutex_frame_queue_);
        std_msgs::Int32 msg;
        msg.data = frame_queue_.size();
        pub_queue_.publish(msg);
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

  void LoopClosing::searchInNeighbors()
  {
    for (int i=c_frame_.getId()-3; i<c_frame_.getId()-1; i++)
    {
      if (i <= 0) continue;
      int inliers = 0;
      tf::Transform edge;
      bool valid = getLoopClosure(c_frame_.getId(), i, edge, inliers);
      if (valid)
      {
        cout << "\033[1;32m[ INFO]: [Localization:] Loop closure (neighbors): " << c_frame_.getId() << " <-> " << i << ".\033[0m\n";
        graph_->addEdge(c_frame_.getId(), i, edge, inliers);
        graph_->update();
      }
    }
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
        cout << "\033[1;32m[ INFO]: [Localization:] Loop closure (proximity): " << c_frame_.getId() << " <-> " << neighbors[i] << ".\033[0m\n";
        graph_->addEdge(c_frame_.getId(), neighbors[i], edge, inliers);
        graph_->update();
      }
    }
  }

  void LoopClosing::searchByHash()
  {
    // Get the candidates to close loop
    vector< pair<int,float> > hash_matching;
    getCandidates(c_frame_.getId(), hash_matching);
    if (hash_matching.size() == 0) return;

    // Check for loop closure
    for (uint i=0; i<hash_matching.size(); i++)
    {
      int inliers = 0;
      tf::Transform edge;
      bool valid = getLoopClosure(c_frame_.getId(), hash_matching[i].first, edge, inliers);
      if (valid)
      {
        cout << "\033[1;32m[ INFO]: [Localization:] Loop closure (hash): " << c_frame_.getId() << " <-> " << hash_matching[i].first << ".\033[0m\n";
        graph_->addEdge(c_frame_.getId(), hash_matching[i].first, edge, inliers);
        graph_->update();
      }
    }
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
    const double ratio = 0.9;
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
                   100, 1.1,
                   MAX_INLIERS, inliers_vector);

    // Inliers threshold
    if (inliers_vector.size() < MIN_INLIERS)
      return false;

    trans = Tools::buildTransformation(rvec, tvec);
    inliers = inliers_vector.size();
    lc_found_.push_back(make_pair(id_a, id_b));
    return true;
  }

  void LoopClosing::getCandidates(int frame_id, vector< pair<int,float> >& candidates)
  {
    // Init
    candidates.clear();

    // Check if enough neighbors
    if ((int)hash_table_.size() <= LC_NEIGHBORS) return;

    // Query matching versus all the hash table
    vector< pair<int,float> > best_matchings;
    getBestMatchings(frame_id, best_matchings);

    // Build the likelihood vector
    vector< pair<int,float> > cur_likelihood;
    buildLikelihoodVector(best_matchings, cur_likelihood);

    // Merge current likelihood with the previous
    vector<float> matchings_likelihood;
    getMatchingsLikelihood(best_matchings, matchings_likelihood, cur_likelihood, prev_likelihood_);

    // Save for the next execution
    prev_likelihood_ = cur_likelihood;

    // Sanity check
    if (best_matchings.size() < 2) return;

    // Group similar images
    vector< vector<int> > groups;
    groupSimilarImages(best_matchings, groups);

    // Order matchings by likelihood
    vector< pair<int, float> > sorted_matchings;
    for (uint i=0; i<best_matchings.size(); i++)
      sorted_matchings.push_back(make_pair(best_matchings[i].first, matchings_likelihood[i]));
    sort(sorted_matchings.begin(), sorted_matchings.end(), Tools::sortByLikelihood);

    // Build the output
    for (uint i=0; i<groups.size(); i++)
    {
      int group_id = -1;
      float group_likelihood = 0.0;
      for (uint j=0; j<groups[i].size(); j++)
      {
        // Search this index into the matchings vector
        for (uint k=0; k<sorted_matchings.size(); k++)
        {
          if (groups[i][j] == sorted_matchings[k].first)
          {
            if (group_id < 0) group_id = groups[i][j];
            group_likelihood += sorted_matchings[k].second;
            break;
          }
        }
      }
      candidates.push_back(make_pair(group_id, group_likelihood));
    }

    // Sort candidates by likelihood
    sort(candidates.begin(), candidates.end(), Tools::sortByLikelihood);
  }

  void LoopClosing::getBestMatchings(int frame_id, vector< pair<int,float> > &best_matchings)
  {
    // Create a list with the non-possible candidates (because they are already loop closings)
    vector<int> no_candidates;
    for (uint i=0; i<lc_found_.size(); i++)
    {
      if (lc_found_[i].first == frame_id)
        no_candidates.push_back(lc_found_[i].second);
      if (lc_found_[i].second == frame_id)
        no_candidates.push_back(lc_found_[i].first);
    }

    // Query hash
    vector<float> hash_q = hash_table_[frame_id].second;

    // Loop over all the hashes stored
    vector< pair<int,float> > all_matchings;
    for (uint i=0; i<hash_table_.size()-LC_NEIGHBORS-1; i++)
    {
      if (i > hash_table_.size()-1 ) continue;

      // Do not compute the hash matching with itself
      if (hash_table_[i].first == frame_id) continue;

      // Continue if candidate is in the no_candidates list
      if (find(no_candidates.begin(), no_candidates.end(), hash_table_[i].first) != no_candidates.end())
        continue;

      // Hash matching
      vector<float> hash_t = hash_table_[i].second;
      float m = hash_.match(hash_q, hash_t);
      all_matchings.push_back(make_pair(hash_table_[i].first, m));
    }

    // Sort the hash matchings
    sort(all_matchings.begin(), all_matchings.end(), Tools::sortByMatching);

    // Retrieve the best n matches
    best_matchings.clear();
    int max_size = 3;
    if (max_size > all_matchings.size()) max_size = all_matchings.size();
    for (uint i=0; i<max_size; i++)
      best_matchings.push_back(all_matchings[i]);
  }

  void LoopClosing::buildLikelihoodVector(vector< pair<int,float> > hash_matchings,
                                          vector< pair<int,float> > &likelihood)
  {
    // Init
    likelihood.clear();

    // Get maximums and minimums of the hash matchings
    int min_idx = -1;
    int max_idx = -1;
    int min_hash = -1;
    int max_hash = -1;
    for (uint i=0; i<hash_matchings.size(); i++)
    {
      if (min_idx < 0 || hash_matchings[i].first < min_idx) min_idx = hash_matchings[i].first;
      if (max_idx < 0 || hash_matchings[i].first > max_idx) max_idx = hash_matchings[i].first;
      if (min_hash < 0 || hash_matchings[i].second < min_hash) min_hash = hash_matchings[i].second;
      if (max_hash < 0 || hash_matchings[i].second > max_hash) max_hash = hash_matchings[i].second;
    }

    // Normalize the hash values
    const float min_norm_val = 1.0;
    const float max_norm_val = 2.0;
    float m = (min_norm_val - max_norm_val) / (max_hash - min_hash);
    float n = max_norm_val - m*min_hash;

    // Build the probability vector
    int space = LC_GROUP_RANGE;
    for (int i=0; i<=(max_idx-min_idx)+2*space; i++)
    {
      int cur_idx = min_idx - space + i;

      // Compute the probability for every candidate
      float prob = 0.0;
      for (uint j=0; j<hash_matchings.size(); j++)
      {
        // Create the normal distribution for this matching
        math::normal_distribution<> nd((float)hash_matchings[j].first, 2.0);

        // Sanity check
        if (!isfinite(m))
          prob += min_norm_val * math::pdf(nd, (float)cur_idx);
        else
          prob += (m*hash_matchings[j].second + n) * math::pdf(nd, (float)cur_idx);
      }
      likelihood.push_back(make_pair(cur_idx,prob));
    }
  }

  void LoopClosing::getMatchingsLikelihood(vector< pair<int,float> > matchings,
                                           vector<float> &matchings_likelihood,
                                           vector< pair<int,float> > cur_likelihood,
                                           vector< pair<int,float> > prev_likelihood)
  {
    // Init
    matchings_likelihood.clear();

    // Extract the vectors
    vector<int> cur_idx;
    for (uint i=0; i<cur_likelihood.size(); i++)
      cur_idx.push_back(cur_likelihood[i].first);

    vector<int> prev_idx;
    for (uint i=0; i<prev_likelihood.size(); i++)
      prev_idx.push_back(prev_likelihood[i].first);

    // For every matching
    for (uint i=0; i<matchings.size(); i++)
    {
      // Find previous value
      float prev_prob = 0.0;
      vector<int>::iterator itp = find(prev_idx.begin(), prev_idx.end(), matchings[i].first);
      if (itp != prev_idx.end())
      {
        int idx = distance(prev_idx.begin(), itp);
        prev_prob = prev_likelihood[idx].second;
      }

      // Find current value
      float cur_prob = 0.0;
      vector<int>::iterator itc = find(cur_idx.begin(), cur_idx.end(), matchings[i].first);
      if (itc != cur_idx.end())
      {
        int idx = distance(cur_idx.begin(), itc);
        cur_prob = cur_likelihood[idx].second;
      }

      // Add and save
      matchings_likelihood.push_back(prev_prob + cur_prob);
    }

    // Make the probability of sum = 1
    float x_norm = 0.0;
    for (uint i=0; i<matchings_likelihood.size(); i++)
      x_norm += fabs(matchings_likelihood[i]);
    for (uint i=0; i<matchings_likelihood.size(); i++)
      matchings_likelihood[i] = matchings_likelihood[i] / x_norm;
  }

  void LoopClosing::groupSimilarImages(vector< pair<int,float> > matchings,
                                       vector< vector<int> > &groups)
  {
    // Init groups vector
    groups.clear();
    vector<int> new_group;
    new_group.push_back(matchings[0].first);
    groups.push_back(new_group);
    matchings.erase(matchings.begin());

    bool finish = false;
    while(!finish)
    {
      // Get the last inserted group
      vector<int> last_group = groups.back();

      // Mean image index
      int sum = accumulate(last_group.begin(), last_group.end(), 0.0);
      float mean = (float)sum / (float)last_group.size();

      bool new_insertion = false;
      for (uint i=0; i<matchings.size(); i++)
      {
        if ( abs(mean - (float)matchings[i].first) < LC_GROUP_RANGE )
        {
          // Save id
          last_group.push_back(matchings[i].first);

          // Replace group
          groups.pop_back();
          groups.push_back(last_group);

          // Delete from matching list
          matchings.erase(matchings.begin() + i);

          new_insertion = true;
          break;
        }
      }

      // Finish?
      if (matchings.size() == 0)
      {
        finish = true;
        continue;
      }

      // Proceed depending on new insertion or not
      if (!new_insertion)
      {
        new_group.clear();
        new_group.push_back(matchings[0].first);
        groups.push_back(new_group);
        matchings.erase(matchings.begin());
      }
    }
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