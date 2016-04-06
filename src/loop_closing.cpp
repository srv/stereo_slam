#include <std_msgs/String.h>
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
    pub_num_keyframes_ = nhp.advertise<std_msgs::String>("keyframes", 2, true);
    pub_num_lc_ = nhp.advertise<std_msgs::String>("loop_closings", 2, true);
    pub_queue_ = nhp.advertise<std_msgs::String>("loop_closing_queue", 2, true);
    pub_lc_matchings_ = nhp.advertise<sensor_msgs::Image>("loop_closing_matchings", 2, true);
  }

  void LoopClosing::run()
  {
    // Init
    execution_dir_ = WORKING_DIRECTORY + "haloc";
    if (fs::is_directory(execution_dir_))
      fs::remove_all(execution_dir_);
    fs::path dir1(execution_dir_);
    if (!fs::create_directory(dir1))
      ROS_ERROR("[Localization:] ERROR -> Impossible to create the loop_closing directory.");

    loop_closures_dir_ = WORKING_DIRECTORY + "loop_closures";
    if (fs::is_directory(loop_closures_dir_))
      fs::remove_all(loop_closures_dir_);
    fs::path dir2(loop_closures_dir_);
    if (!fs::create_directory(dir2))
      ROS_ERROR("[Localization:] ERROR -> Impossible to create the loop_closing directory.");

    camera_model_ = graph_->getCameraModel();
    num_loop_closures_ = 0;

    // Publish first loop closure image
    stringstream s;
    s << " No Loop Closures ";
    cv::Mat no_loops = cv::Mat(384, 512, CV_8UC3);
    no_loops.rowRange(0, no_loops.rows).setTo(cv::Scalar(0,0,0));
    cv::putText(no_loops, s.str(), cv::Point(95, 200), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255,255,255), 2, 8);
    cv_bridge::CvImage ros_image;
    ros_image.image = no_loops.clone();
    ros_image.header.stamp = ros::Time::now();
    ros_image.encoding = "bgr8";
    pub_lc_matchings_.publish(ros_image.toImageMsg());

    // Loop
    ros::Rate r(50);
    while(ros::ok())
    {

      // Process new frame
      if(checkNewClusterInQueue())
      {
        processNewCluster();

        searchByProximity();

        searchByHash();

        // Publish loop closing information
        if (pub_num_keyframes_.getNumSubscribers() > 0)
        {
          std_msgs::String msg;
          msg.data = lexical_cast<string>(graph_->getFrameNum());
          pub_num_keyframes_.publish(msg);
        }
        if (pub_num_lc_.getNumSubscribers() > 0)
        {
          std_msgs::String msg;
          msg.data = lexical_cast<string>(cluster_lc_found_.size());
          pub_num_lc_.publish(msg);
        }
        if (pub_queue_.getNumSubscribers() > 0)
        {
          mutex::scoped_lock lock(mutex_cluster_queue_);
          std_msgs::String msg;
          msg.data = lexical_cast<string>(cluster_queue_.size());
          pub_queue_.publish(msg);
        }
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
    write(fs, "kp_l", c_cluster_.getLeftKp());
    write(fs, "kp_r", c_cluster_.getRightKp());
    write(fs, "desc", c_cluster_.getOrb());
    write(fs, "points", c_cluster_.getPoints());
    fs.release();
  }

  void LoopClosing::searchByProximity()
  {
    vector<int> cand_neighbors;
    graph_->findClosestVertices(c_cluster_.getId(), c_cluster_.getId(), LC_DISCARD_WINDOW, 3, cand_neighbors);
    for (uint i=0; i<cand_neighbors.size(); i++)
    {
      Cluster candidate = readCluster(cand_neighbors[i]);
      if (candidate.getOrb().rows == 0)
        continue;

      bool valid = closeLoopWithCluster(candidate);

      if (valid)
        ROS_INFO("[Localization:] Loop closure by proximity.");
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
      Cluster candidate = readCluster(hash_matching[i].first);
      if (candidate.getOrb().rows == 0)
        continue;

      bool valid = closeLoopWithCluster(candidate);

      if (valid)
      {
        ROS_INFO("[Localization:] Loop closure by hash.");
        break;
      }
    }
  }

  bool LoopClosing::closeLoopWithCluster(Cluster candidate)
  {
    // Init
    const float matching_th = 0.8;

    // Descriptor matching
    vector<cv::DMatch> matches_1;
    Tools::ratioMatching(c_cluster_.getOrb(), candidate.getOrb(), matching_th, matches_1);

    // Compute the percentage of matchings
    int size_c = c_cluster_.getOrb().rows;
    int size_n = candidate.getOrb().rows;
    int m_percentage = round(100.0 * (float) matches_1.size() / (float) min(size_c, size_n) );

    // Get the neighbor clusters if enough matching percentage
    if (m_percentage > 35)
    {
      // Init accumulated data
      vector<int> cluster_query_list;
      vector<int> cluster_cand_list;

      // Candidate data
      cv::Mat all_cand_desc = candidate.getOrb();
      vector<cv::Point3f> all_cand_points = candidate.getWorldPoints();
      vector<cv::KeyPoint> all_cand_kp_l = candidate.getLeftKp();
      vector<cv::KeyPoint> all_cand_kp_r = candidate.getRightKp();

      // Query data
      cv::Mat all_query_desc = c_cluster_.getOrb();
      vector<cv::KeyPoint> all_query_kp_l = c_cluster_.getLeftKp();
      vector<cv::KeyPoint> all_query_kp_r = c_cluster_.getRightKp();

      // Init the cluster candidate list
      for (uint j=0; j<all_cand_points.size(); j++)
        cluster_cand_list.push_back(candidate.getId());

      // Increase the probability to close loop by extracting the candidate neighbors
      vector<int> cand_neighbors;
      graph_->findClosestVertices(candidate.getId(), c_cluster_.getId(), LC_DISCARD_WINDOW, LC_NEIGHBORS, cand_neighbors);
      for (uint j=0; j<cand_neighbors.size(); j++)
      {
        Cluster cand_neighbor = readCluster(cand_neighbors[j]);
        cv::Mat c_n_desc = cand_neighbor.getOrb();
        if (c_n_desc.rows == 0) continue;

        vector<cv::Point3f> points_tmp = cand_neighbor.getWorldPoints();
        vector<cv::KeyPoint> kp_tmp_l = cand_neighbor.getLeftKp();
        vector<cv::KeyPoint> kp_tmp_r = cand_neighbor.getRightKp();

        // Concatenate descriptors and points
        cv::vconcat(all_cand_desc, c_n_desc, all_cand_desc);
        all_cand_points.insert(all_cand_points.end(), points_tmp.begin(), points_tmp.end());
        all_cand_kp_l.insert(all_cand_kp_l.end(), kp_tmp_l.begin(), kp_tmp_l.end());
        all_cand_kp_r.insert(all_cand_kp_r.end(), kp_tmp_r.begin(), kp_tmp_r.end());

        // Save the cluster id for these points
        for (uint n=0; n<points_tmp.size(); n++)
          cluster_cand_list.push_back(cand_neighbor.getId());
      }

      // Init the cluster frame list
      for (uint j=0; j<c_cluster_.getLeftKp().size(); j++)
        cluster_query_list.push_back(c_cluster_.getId());

      // Extract all the clusters corresponding to the current cluster frame
      vector<int> query_clusters;
      graph_->getFrameVertices(c_cluster_.getFrameId(), query_clusters);
      for (uint j=0; j<query_clusters.size(); j++)
      {
        if (query_clusters[j] == c_cluster_.getId())
          continue;

        Cluster query_cluster = readCluster(query_clusters[j]);
        cv::Mat f_n_desc = query_cluster.getOrb();
        if (f_n_desc.rows == 0) continue;

        // Concatenate descriptors and keypoints
        cv::vconcat(all_query_desc, f_n_desc, all_query_desc);
        vector<cv::KeyPoint> query_n_kp_l = query_cluster.getLeftKp();
        vector<cv::KeyPoint> query_n_kp_r = query_cluster.getRightKp();
        all_query_kp_l.insert(all_query_kp_l.end(), query_n_kp_l.begin(), query_n_kp_l.end());
        all_query_kp_r.insert(all_query_kp_r.end(), query_n_kp_r.begin(), query_n_kp_r.end());

        // Save the cluster id for these points
        for (uint n=0; n<query_n_kp_l.size(); n++)
          cluster_query_list.push_back(query_cluster.getId());
      }

      // Match current frame descriptors with all the clusters
      vector<cv::DMatch> matches_2;
      Tools::ratioMatching(all_query_desc, all_cand_desc, matching_th, matches_2);

      if (matches_2.size() >= LC_MIN_INLIERS)
      {
        // Store matchings
        vector<int> query_matchings;
        vector<int> cand_matchings;
        vector<cv::Point2f> matched_query_kp_l, matched_query_kp_r;
        vector<cv::Point2f> matched_cand_kp_l, matched_cand_kp_r;
        vector<cv::Point3f> matched_cand_3d_points;
        for(uint j=0; j<matches_2.size(); j++)
        {
          // Features
          matched_query_kp_l.push_back(all_query_kp_l[matches_2[j].queryIdx].pt);
          matched_query_kp_r.push_back(all_query_kp_r[matches_2[j].queryIdx].pt);
          matched_cand_kp_l.push_back(all_cand_kp_l[matches_2[j].trainIdx].pt);
          matched_cand_kp_r.push_back(all_cand_kp_r[matches_2[j].trainIdx].pt);

          // 3d
          matched_cand_3d_points.push_back(all_cand_points[matches_2[j].trainIdx]);

          // Ids
          query_matchings.push_back(cluster_query_list[matches_2[j].queryIdx]);
          cand_matchings.push_back(cluster_cand_list[matches_2[j].trainIdx]);
        }

        // Estimate the motion
        vector<int> inliers;
        cv::Mat rvec, tvec;
        cv::solvePnPRansac(matched_cand_3d_points, matched_query_kp_l, graph_->getCameraMatrix(),
            cv::Mat(), rvec, tvec, false,
            100, 4.0, LC_MAX_INLIERS, inliers);

        // Loop found!
        if (inliers.size() >= LC_MIN_INLIERS)
        {
          tf::Transform estimated_transform = Tools::buildTransformation(rvec, tvec);
          estimated_transform = estimated_transform.inverse();

          // Get the inliers per cluster pair
          vector< vector<int> > cluster_pairs;
          vector<int> inliers_per_pair;
          vector<int> cand_kfs;
          for (uint i=0; i<inliers.size(); i++)
          {
            int query_cluster = query_matchings[inliers[i]];
            int cand_cluster = cand_matchings[inliers[i]];
            int cand_kf = graph_->getVertexFrameId(cand_cluster);

            // Build the unique candidate keyframes vector
            bool found_0 = false;
            for (uint j=0; j<cand_kfs.size(); j++)
            {
              if (cand_kfs[j] == cand_kf)
              {
                found_0 = true;
                break;
              }
            }
            if (!found_0)
              cand_kfs.push_back(cand_kf);

            // Search if this pair already exists
            bool found_1 = false;
            uint idx = 0;
            for (uint j=0; j<cluster_pairs.size(); j++)
            {
              if (cluster_pairs[j][0] == query_cluster && cluster_pairs[j][1] == cand_cluster)
              {
                found_1 = true;
                idx = j;
                break;
              }
            }

            if (found_1)
              inliers_per_pair[idx]++;
            else
            {
              vector<int> pair;
              pair.push_back(query_cluster);
              pair.push_back(cand_cluster);
              cluster_pairs.push_back(pair);
              inliers_per_pair.push_back(1);
            }
          }

          // Add the corresponding edges
          vector< vector<int> > definitive_cluster_pairs;
          vector<int> definitive_inliers_per_pair;
          bool some_edge_added = false;
          for (uint i=0; i<inliers_per_pair.size(); i++)
          {
            if (inliers_per_pair[i] >= 5)
            {
              // Check if this frame exists
              bool lc_found = false;
              for (uint l=0; l<cluster_lc_found_.size(); l++)
              {
                if ( (cluster_lc_found_[l].first == cluster_pairs[i][0] && cluster_lc_found_[l].second == cluster_pairs[i][1]) ||
                    (cluster_lc_found_[l].first == cluster_pairs[i][1] && cluster_lc_found_[l].second == cluster_pairs[i][0]) )
                {
                  lc_found = true;
                  break;
                }
              }
              if (lc_found) continue;

              tf::Transform candidate_cluster_pose = graph_->getVertexPose(cluster_pairs[i][1]);
              tf::Transform frame_cluster_pose_relative_to_camera = graph_->getVertexPoseRelativeToCamera(cluster_pairs[i][0]);
              tf::Transform edge_1 = candidate_cluster_pose.inverse() * estimated_transform * frame_cluster_pose_relative_to_camera;

              // Add this edge to the graph
              graph_->addEdge(cluster_pairs[i][1], cluster_pairs[i][0], edge_1, inliers_per_pair[i]);
              vector<int> pair;
              pair.push_back(cluster_pairs[i][0]);
              pair.push_back(cluster_pairs[i][1]);
              definitive_cluster_pairs.push_back(pair);
              definitive_inliers_per_pair.push_back(inliers_per_pair[i]);
              some_edge_added = true;

              // Add this edge to the cluster list of loop closings found
              cluster_lc_found_.push_back(make_pair(cluster_pairs[i][0], cluster_pairs[i][1]));
            }
          }

          if (some_edge_added)
          {
            num_loop_closures_++;

            // Update the graph with the new edges
            graph_->update();

            // Draw the loop closure to image
            drawLoopClosure(cand_kfs,
                            cand_matchings,
                            inliers,
                            definitive_inliers_per_pair,
                            definitive_cluster_pairs,
                            matched_query_kp_l,
                            matched_cand_kp_l);
            return true;
          }
        }
      }
    }

    return false;
  }

  void LoopClosing::getCandidates(int cluster_id, vector< pair<int,float> >& candidates)
  {
    // Init
    candidates.clear();

    // Check if enough neighbors
    if ((int)hash_table_.size() <= LC_DISCARD_WINDOW) return;

    // Create a list with the non-possible candidates (because they are already loop closings)
    vector<int> no_candidates;
    for (uint i=0; i<cluster_lc_found_.size(); i++)
    {
      if (cluster_lc_found_[i].first == cluster_id)
        no_candidates.push_back(cluster_lc_found_[i].second);
      if (cluster_lc_found_[i].second == cluster_id)
        no_candidates.push_back(cluster_lc_found_[i].first);
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
    vector<cv::KeyPoint> kp_l, kp_r;
    cv::Mat desc, pose, empty;
    vector<cv::Point3f> points;
    fs["frame_id"] >> frame_id;
    fs["desc"] >> desc;
    fs["points"] >> points;
    cv::FileNode kp_l_node = fs["kp_l"];
    cv::FileNode kp_r_node = fs["kp_r"];
    read(kp_l_node, kp_l);
    read(kp_r_node, kp_r);
    fs.release();

    // Set the properties of the cluster
    tf::Transform vertex_camera_pose = graph_->getVertexCameraPose(id, true);
    Cluster cluster_tmp(id, frame_id, vertex_camera_pose, kp_l, kp_r, desc, empty, points);
    cluster = cluster_tmp;

    return cluster;
  }

  void LoopClosing::drawLoopClosure(vector<int> cand_kfs,
                                    vector<int> cand_matchings,
                                    vector<int> inliers,
                                    vector<int> definitive_inliers_per_pair,
                                    vector< vector<int> > definitive_cluster_pairs,
                                    vector<cv::Point2f> matched_query_kp_l,
                                    vector<cv::Point2f> matched_cand_kp_l)
  {
    // Build image with loop closure matchings
    cv::Mat lc_image;

    // Read candidate keyframes
    vector<int> idx_img_candidate_kfs;
    cv::Mat img_candidate_kfs;
    int baseline = 0;
    for (uint i=0; i<cand_kfs.size(); i++)
    {
      string frame_id_str = Tools::convertTo5digits(cand_kfs[i]);
      string keyframe_file = WORKING_DIRECTORY + "keyframes/" + frame_id_str + ".jpg";
      cv::Mat kf = cv::imread(keyframe_file, CV_LOAD_IMAGE_COLOR);

      // Add the keyframe identifier
      stringstream s;
      s << " Keyframe " << cand_kfs[i];
      cv::Size text_size = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
      cv::Mat im_text = cv::Mat(kf.rows + text_size.height + 10, kf.cols, kf.type());
      kf.copyTo(im_text.rowRange(0, kf.rows).colRange(0, kf.cols));
      im_text.rowRange(kf.rows, im_text.rows).setTo(cv::Scalar(255,255,255));
      cv::putText(im_text, s.str(), cv::Point(5, im_text.rows - 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0), 1, 8);

      if (img_candidate_kfs.cols == 0)
        im_text.copyTo(img_candidate_kfs);
      else
        cv::hconcat(img_candidate_kfs, im_text, img_candidate_kfs);

      // Store the index
      idx_img_candidate_kfs.push_back(cand_kfs[i]);
    }

    // Read the current keyframe
    string frame_id_str = Tools::convertTo5digits(c_cluster_.getFrameId());
    string keyframe_file = WORKING_DIRECTORY + "keyframes/" + frame_id_str + ".jpg";
    cv::Mat current_kf_tmp = cv::imread(keyframe_file, CV_LOAD_IMAGE_COLOR);

    // Add the keyframe identifier
    stringstream s;
    s << " Keyframe " << c_cluster_.getFrameId();
    cv::Size text_size = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
    cv::Mat current_kf_text = cv::Mat(current_kf_tmp.rows + text_size.height + 10, current_kf_tmp.cols, current_kf_tmp.type());
    current_kf_tmp.copyTo(current_kf_text.rowRange(text_size.height + 10, current_kf_tmp.rows + text_size.height + 10).colRange(0, current_kf_tmp.cols));
    current_kf_text.rowRange(0, text_size.height + 10).setTo(cv::Scalar(255,255,255));
    cv::putText(current_kf_text, s.str(), cv::Point(5, 14), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0), 1, 8);

    // Compose current keyframe with candidate keyframes
    cv::Mat current_kf(img_candidate_kfs.rows, img_candidate_kfs.cols, img_candidate_kfs.type(), cv::Scalar(0, 0, 0));
    int x_offset = round(img_candidate_kfs.cols/2 - current_kf_text.cols/2);
    current_kf_text.copyTo( current_kf( cv::Rect(x_offset, 0, current_kf_text.cols, current_kf_text.rows) ) );
    cv::vconcat(current_kf, img_candidate_kfs, lc_image);

    // Build the vector of colors
    cv::RNG rng(12345);
    vector<cv::Scalar> colors;
    for (uint i=0; i<definitive_inliers_per_pair.size(); i++)
    {
      cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
      colors.push_back(color);
    }

    // Draw the matchings
    for (uint i=0; i<inliers.size(); i++)
    {
      // Extract the keypoint for the current keyframe
      cv::Point2f current_kp = matched_query_kp_l[inliers[i]];
      cv::Point2f candidate_kp = matched_cand_kp_l[inliers[i]];

      // Candidate cluster identifier
      int cand_cluster = cand_matchings[inliers[i]];
      int cand_keyframe = graph_->getVertexFrameId(cand_cluster);

      int position = -1;
      for (uint j=0; j<idx_img_candidate_kfs.size(); j++)
      {
        if (idx_img_candidate_kfs[j] == cand_keyframe)
        {
          position = j;
          break;
        }
      }

      // Draw
      if (position >= 0)
      {
        // Decide the color
        int color_idx = -1;
        for (uint n=0; n<definitive_inliers_per_pair.size(); n++)
        {
          if (definitive_cluster_pairs[n][1] == cand_cluster)
          {
            color_idx = n;
            break;
          }
        }
        if (color_idx >= 0)
        {
          cv::Point2f p_cur_kf = current_kp;
          p_cur_kf.x = x_offset + p_cur_kf.x;

          cv::Point2f p_cand_kf = candidate_kp;
          p_cand_kf.y = p_cand_kf.y + current_kf.rows;
          p_cand_kf.x = position*current_kf_text.cols + p_cand_kf.x;

          cv::circle(lc_image, p_cur_kf, 4, colors[color_idx], -1);
          cv::circle(lc_image, p_cand_kf, 4, colors[color_idx], -1);
          cv::line(lc_image, p_cur_kf, p_cand_kf, colors[color_idx], 2, 8, 0);
        }
      }
    }

    // Save
    string lc_file = loop_closures_dir_ + "/" + Tools::convertTo5digits(num_loop_closures_) + ".jpg";
    cv::imwrite( lc_file, lc_image );

    // Publish
    if (pub_lc_matchings_.getNumSubscribers() > 0)
    {
      cv_bridge::CvImage ros_image;
      ros_image.image = lc_image.clone();
      ros_image.header.stamp = ros::Time::now();
      ros_image.encoding = "bgr8";
      pub_lc_matchings_.publish(ros_image.toImageMsg());
    }
  }

  void LoopClosing::finalize()
  {
    // Remove the temporal directory
    if (fs::is_directory(execution_dir_))
      fs::remove_all(execution_dir_);
  }

} //namespace slam
