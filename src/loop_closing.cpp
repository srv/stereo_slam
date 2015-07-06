#include <std_msgs/Int32.h>
#include <image_geometry/pinhole_camera_model.h>

#include <numeric>

#include "loop_closing.h"
#include "tools.h"

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

    camera_model_ = graph_->getCameraModel();

    // Loop
    ros::Rate r(500);
    while(ros::ok())
    {
      if(checkNewClusterInQueue())
      {
        processNewCluster();

        searchInPreviousNeighors();

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
        mutex::scoped_lock lock(mutex_cluster_queue_);
        std_msgs::Int32 msg;
        msg.data = cluster_queue_.size();
        pub_queue_.publish(msg);
      }

      r.sleep();
    }
  }

  void LoopClosing::addClusterToQueue(Cluster cluster)
  {
    mutex::scoped_lock lock(mutex_cluster_queue_);
    cluster_queue_.push_back(cluster);
  }

  bool LoopClosing::checkNewClusterInQueue()
  {
    mutex::scoped_lock lock(mutex_cluster_queue_);
    return(!cluster_queue_.empty());
  }

  void LoopClosing::processNewCluster()
  {
    // Get the cluster
    {
      mutex::scoped_lock lock(mutex_cluster_queue_);
      c_cluster_ = cluster_queue_.front();
      cluster_queue_.pop_front();
    }

    // Initialize hash
    if (!hash_.isInitialized())
      hash_.init(c_cluster_.getSift());

    // Save hash to table
    hash_table_.push_back(make_pair(c_cluster_.getId(), hash_.getHash(c_cluster_.getSift())));

    // Store
    cv::FileStorage fs(execution_dir_+"/"+lexical_cast<string>(c_cluster_.getId())+".yml", cv::FileStorage::WRITE);
    write(fs, "frame_id", c_cluster_.getFrameId());
    write(fs, "pose", Tools::transformToMat(c_cluster_.getPose()));
    write(fs, "kp", c_cluster_.getKp());
    write(fs, "desc", c_cluster_.getLdb());
    write(fs, "points", c_cluster_.getPoints());
    fs.release();
  }

  void LoopClosing::searchInPreviousNeighors()
  {
    // Init
    int id = c_cluster_.getId();
    int current_frame_id = c_cluster_.getFrameId();
    int processed_neighbors = 0;
    int neighbor_id = id - 1;
    vector<cv::KeyPoint> kp = c_cluster_.getKp();

    // Store matchings
    vector<cv::Point2f> matched_kp;
    vector<cv::Point3f> matched_points;

    // Loop over neighbors of different frames
    while (processed_neighbors < LC_NEIGHBORS && neighbor_id >= 0)
    {
      // Extract the neighbor
      Cluster neighbor = readCluster(neighbor_id);
      vector<cv::Point3f> neighbor_points = neighbor.getPoints();

      if (neighbor.getFrameId() != current_frame_id)
      {
        vector<cv::DMatch> matches;
        Tools::ratioMatching(c_cluster_.getLdb(), neighbor.getLdb(), 0.8, matches);

        // Compute the percentage of matchings
        int size_c = c_cluster_.getLdb().rows;
        int size_n = neighbor.getLdb().rows;
        int m_percentage = round(100.0 * (float) matches.size() / (float) min(size_c, size_n) );

        // Only consider clusters with high number of matchings
        if (m_percentage > 50)
        {
          for(uint i=0; i<matches.size(); i++)
          {
            matched_kp.push_back(kp[matches[i].queryIdx].pt);

            // Transform camera point to world coordinates
            cv::Point3f p_world = Tools::transformPoint(neighbor_points[matches[i].trainIdx], neighbor.getPose());
            matched_points.push_back(p_world);
          }
        }
        processed_neighbors++;
      }
      neighbor_id--;
    }

    // Compute cluster world pose
    if (matched_points.size() > 10)
    {
      // TODO: Add the edges between neighbor clusters

      // // Estimate the motion
      // vector<int> inliers;
      // cv::Mat rvec, tvec;
      // solvePnPRansac(matched_points, matched_kp, graph_->getCameraMatrix(),
      //                cv::Mat(), rvec, tvec, false,
      //                100, 2.0, MAX_INLIERS, inliers);

      // ROS_INFO_STREAM("KKK: " << matched_points.size() << ", " << inliers.size());
    }
  }

  void LoopClosing::searchByHash()
  {
    // Get the candidates to close loop
    vector< pair<int,float> > hash_matching;
    getCandidates(c_cluster_.getId(), hash_matching);
    if (hash_matching.size() == 0) return;

    // Loop over candidates
    for (uint i=0; i<hash_matching.size(); i++)
    {
      // Get the candidate
      int candidate_id = hash_matching[i].first;
      Cluster candidate = readCluster(candidate_id);
      string frame_list = boost::lexical_cast<string>( candidate.getFrameId() );
      string cluster_list = boost::lexical_cast<string>( candidate.getId() );

      // Descriptor matching
      vector<cv::DMatch> matches_1;
      Tools::ratioMatching(c_cluster_.getLdb(), candidate.getLdb(), 0.8, matches_1);

      // Compute the percentage of matchings
      int size_c = c_cluster_.getLdb().rows;
      int size_n = candidate.getLdb().rows;
      int m_percentage = round(100.0 * (float) matches_1.size() / (float) min(size_c, size_n) );

      // Get the neighbor clusters if high percentage of matching
      if (m_percentage > 40)
      {
        // Init accumulated data
        cv::Mat all_candidate_desc;
        vector<cv::Point3f> all_candidate_points;
        vector<int> cluster_neightbor_list;
        cv::Mat all_frame_desc = c_cluster_.getLdb();
        vector<cv::KeyPoint> all_frame_kp = c_cluster_.getKp();

        // Check if 3D points of the candidate are in camera frustum
        filterByFrustum(candidate.getLdb(), candidate.getWorldPoints(), c_cluster_.getPose(), all_candidate_desc, all_candidate_points);

        for (uint j=0; j<all_candidate_points.size(); j++)
          cluster_neightbor_list.push_back(candidate.getId());

        // Increase the probability to close loop by extracting the candidate neighbors
        vector<int> candidate_neighbors;
        graph_->findClosestVertices(candidate_id, c_cluster_.getId(), LC_DISCARD_WINDOW, 5, candidate_neighbors);
        for (uint j=0; j<candidate_neighbors.size(); j++)
        {
          Cluster candidate_neighbor = readCluster(candidate_neighbors[j]);
          cv::Mat c_n_desc = candidate_neighbor.getLdb();
          if (c_n_desc.rows == 0) continue;

          frame_list += ", " + boost::lexical_cast<string>( candidate_neighbor.getFrameId() );
          cluster_list += ", " + boost::lexical_cast<string>( candidate_neighbor.getId() );

          // Delete points outside of frustum
          cv::Mat desc_tmp;
          vector<cv::Point3f> points_tmp;
          filterByFrustum(c_n_desc, candidate_neighbor.getWorldPoints(), c_cluster_.getPose(), desc_tmp, points_tmp);

          // Concatenate descriptors and points
          cv::vconcat(all_candidate_desc, desc_tmp, all_candidate_desc);
          all_candidate_points.insert(all_candidate_points.end(), points_tmp.begin(), points_tmp.end());

          for (uint n=0; n<points_tmp.size(); n++)
            cluster_neightbor_list.push_back(candidate_neighbor.getId());
        }

        // Extract all the clusters corresponding to the current cluster frame
        vector<int> frame_clusters;
        graph_->getFrameVertices(c_cluster_.getFrameId(), frame_clusters);
        for (uint j=0; j<frame_clusters.size(); j++)
        {
          if (frame_clusters[j] == c_cluster_.getId())
            continue;

          Cluster frame_cluster = readCluster(frame_clusters[j]);
          cv::Mat f_n_desc = frame_cluster.getLdb();
          if (f_n_desc.rows == 0) continue;

          // Concatenate descriptors and keypoints
          vector<cv::KeyPoint> frame_n_kp = frame_cluster.getKp();
          cv::vconcat(all_frame_desc, f_n_desc, all_frame_desc);
          all_frame_kp.insert(all_frame_kp.end(), frame_n_kp.begin(), frame_n_kp.end());
        }

        // Match current frame descriptors with all the clusters
        vector<cv::DMatch> matches_2;
        Tools::ratioMatching(all_frame_desc, all_candidate_desc, 0.8, matches_2);

        if (matches_2.size() > 5)
        {
          // Store matchings
          vector<int> cluster_matchings;
          vector<cv::Point2f> matched_kp;
          vector<cv::Point3f> matched_points;
          for(uint j=0; j<matches_2.size(); j++)
          {
            matched_kp.push_back(all_frame_kp[matches_2[j].queryIdx].pt);
            matched_points.push_back(all_candidate_points[matches_2[j].trainIdx]);
            cluster_matchings.push_back(cluster_neightbor_list[matches_2[j].trainIdx]);
          }

          // Get the list of affected clusters
          string c_matchings = "";
          vector<int> cluster_matchings_unique = cluster_matchings;;
          sort(cluster_matchings_unique.begin(), cluster_matchings_unique.end());
          vector<int>::iterator last = unique(cluster_matchings_unique.begin(), cluster_matchings_unique.end());
          cluster_matchings_unique.erase(last, cluster_matchings_unique.end());
          for (uint j=0; j<cluster_matchings_unique.size(); j++)
          {
            int matchings = count(cluster_matchings.begin(), cluster_matchings.end(), cluster_matchings_unique[j]);
            c_matchings += boost::lexical_cast<string>(cluster_matchings_unique[j]) + " -> " + boost::lexical_cast<string>(matchings) + ", ";
          }

          // Estimate the motion
          vector<int> inliers;
          cv::Mat rvec, tvec;
          solvePnPRansac(matched_points, matched_kp, graph_->getCameraMatrix(),
                         cv::Mat(), rvec, tvec, false,
                         100, 1.0, MAX_INLIERS, inliers);

          if (inliers.size() > 1)
          {
            tf::Transform pose = Tools::buildTransformation(rvec, tvec);
            pose = pose.inverse();

            ROS_INFO_STREAM("LOOP! " << c_cluster_.getFrameId() << " <-> [Frames: " << frame_list << "] Matches: " << matches_2.size() << ", Inliers: " << inliers.size());
            ROS_INFO_STREAM("LOOP! " << c_cluster_.getId() << " <-> [Clusters: " << cluster_list << "] Matches: " << matches_2.size() << ", Inliers: " << inliers.size());
            ROS_INFO_STREAM("MATCHES PER CLUSTER: " << c_matchings);
            ROS_INFO_STREAM("ODOM: " << c_cluster_.getPose().getOrigin().x() << ", " << c_cluster_.getPose().getOrigin().y() << ", " << c_cluster_.getPose().getOrigin().z());
            ROS_INFO_STREAM("SPNP: " << pose.getOrigin().x() << ", " << pose.getOrigin().y() << ", " << pose.getOrigin().z());
            cout << endl;
            cout << endl;
          }
        }
      }
    }
  }

  bool LoopClosing::isInFrustum(cv::Point3f point, tf::Transform camera_pose)
  {
    cv::Size res = camera_model_.fullResolution();
    cv::Point3f p_camera = Tools::transformPoint(point, camera_pose.inverse());
    cv::Point2d pixels = camera_model_.project3dToPixel(p_camera);
    if (pixels.x < 0 || pixels.x > res.width) return false;
    if (pixels.y < 0 || pixels.y > res.height) return false;
    return true;
  }

  void LoopClosing::filterByFrustum(cv::Mat desc, vector<cv::Point3f> points, tf::Transform camera_pose, cv::Mat& out_desc, vector<cv::Point3f>& out_points)
  {
    out_desc.release();
    out_points.clear();

    for (uint i=0; i<points.size(); i++)
    {
      cv::Point3f p = points[i];
      if ( isInFrustum(p, camera_pose) )
      {
        out_desc.push_back(desc.row(i));
        out_points.push_back(p);
      }
    }
  }

  void LoopClosing::getCandidates(int cluster_id, vector< pair<int,float> >& candidates)
  {
    // Init
    candidates.clear();

    // Check if enough neighbors
    if ((int)hash_table_.size() <= LC_DISCARD_WINDOW) return;

    // Create a list with the non-possible candidates (because they are already loop closings)
    vector<int> no_candidates;
    for (uint i=0; i<lc_found_.size(); i++)
    {
      if (lc_found_[i].first == cluster_id)
        no_candidates.push_back(lc_found_[i].second);
      if (lc_found_[i].second == cluster_id)
        no_candidates.push_back(lc_found_[i].first);
    }

    // Query hash
    vector<float> hash_q = hash_table_[cluster_id].second;

    // Loop over all the hashes stored
    vector< pair<int,float> > all_matchings;
    for (uint i=0; i<hash_table_.size(); i++)
    {
      // Discard window
      if (hash_table_[i].first > cluster_id-LC_DISCARD_WINDOW && hash_table_[i].first < cluster_id+LC_DISCARD_WINDOW) continue;

      // Do not compute the hash matching with itself
      if (hash_table_[i].first == cluster_id) continue;

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
    uint max_size = 5;
    if (max_size > all_matchings.size()) max_size = all_matchings.size();
    for (uint i=0; i<max_size; i++)
      candidates.push_back(all_matchings[i]);
  }

  Cluster LoopClosing::readCluster(int id)
  {
    string file = execution_dir_+"/"+lexical_cast<string>(id)+".yml";
    Cluster cluster;

    // Sanity check
    if ( !fs::exists(file) ) return cluster;

    cv::FileStorage fs;
    fs.open(file, cv::FileStorage::READ);
    if (!fs.isOpened()) return cluster;

    int frame_id;
    vector<cv::KeyPoint> kp;
    cv::Mat desc, pose, empty;
    vector<cv::Point3f> points;
    fs["frame_id"] >> frame_id;
    fs["pose"] >> pose;
    fs["desc"] >> desc;
    fs["points"] >> points;
    cv::FileNode kp_node = fs["kp"];
    read(kp_node, kp);
    fs.release();

    // Set the properties of the cluster
    Cluster cluster_tmp(id, frame_id, Tools::matToTransform(pose), kp, desc, empty, points);
    cluster = cluster_tmp;

    return cluster;
  }

  void LoopClosing::finalize()
  {
    // Remove the temporal directory
    if (fs::is_directory(execution_dir_))
      fs::remove_all(execution_dir_);
  }

} //namespace slam