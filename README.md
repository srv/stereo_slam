Stereo SLAM
=============

stereo_slam is a [ROS][link_ros] node to execute Simultaneous Localization And Mapping (SLAM) using only one stereo camera. The algorithm was designed and tested for underwater robotics. This node is based on the [G2O][link_g2o] library for graph optimization and uses the power of PostgreSQL to store the image keypoints and descriptors. The workflow of the stereo_slam node is as follows:

![alt tag](https://raw.github.com/srv/stereo_slam/fuerte/resources/flowchart.png)

You can see it in action here:
[Stereo SLAM and 3D reconstruction ][link_yt_1]
[Stereo SLAM at UIB outdoor pond][link_yt_2]

Installation (Ubuntu + ROS fuerte)
-------

1) Install the dependencies
```bash
sudo apt-get install ros-fuerte-libg2o
sudo apt-get install ros-fuerte-sql-database
sudo apt-get install postgresql pgadmin3
```
You also need to setup a stereo visual odometer (e.g. [viso2][link_viso2] or [fovis][link_fovis]).

2) Configure the PostgreSQL database. This creates the user postgres with password postgres. You can setup a different user and password but then you must specify these parameters to the stereo_slam ROS node.
```bash
sudo -u postgres psql postgres
\password postgres
```

3) Download the code, save it to your ROS workspace enviroment and compile.
```bash
roscd
git clone https://github.com/srv/stereo_slam.git
cd stereo_slam
rosmake
```


Parameters
-------

### Most important ###

* `odom_topic` - Visual odometry topic (type nav_msgs::Odometry).
* `left_topic` - Left image camera topic (type sensor_msgs::Image).
* `right_topic` - Right image camera topic (type sensor_msgs::Image).
* `left_info_topic` - Left info camera topic (type sensor_msgs::CameraInfo).
* `right_info_topic` - Right info camera topic (type sensor_msgs::CameraInfo).
* `min_displacement` - Min displacement between graph vertices (in meters).
* `max_candidate_threshold` - Loop closing candidate search radius (in meters).
* `update_rate` - G2O update rate (in seconds).
* `descriptor_threshold` - Descriptor threshold (for SIFT typically between 0.8-0.9).
* `matches_threshold` - Minimum number of matches between loop closing candidates. (Step 1 of the candidate search stage). If you don't have loop closings, try to decrese this parameter.
* `min_inliers` - Minimum number of inliers between loop closing candidates (Step 2 of the candidate search stage). If you don't have loop closings, try to decrese this parameter (minimum value = 7).
* `max_edge_err` - Maximum allowed error between loop closing candidates (Step 3 of the candidate search stage).
* `save_graph_to_file` - true to save the graph to an output file.
* `files_path` - Path where the graphvertices.txt and graphedges.txt files are saved.
* `go2_opt_max_iter` - Maximum number of g2o alogirthm iterations (typically between 10-50)

### Other parameters (do not touch by default) ###

#### PostgreSQL Database ####
* `db_host` - url to the PostgreSQL database server (localhost by default).
* `db_port` - PostgreSQL port (5432 by default).
* `db_user` - PostgreSQL user (postgres by default).
* `db_pass` - PostgreSQL password (postgres by default).
* `db_name` - Database name (graph by default).

#### G2O Library ####
* `g2o_algorithm` - Set to 0 for LinearSlam Solver with gauss-newton. Set to 1 for LinearSlam Solver with Levenberg (Default 1).
* `go2_verbose` - True to output the g2o iteration messages.

#### Graph ####
* `neighbor_offset` - Number of neighbor graph vertices discarted for loop-closing.

#### Stereo vision ####
* `desc_type` - Can be SIFT or SURF (SIFT by default).
* `epipolar_threshold` - Maximum epipolar distance for stereo matching.
* `max_inliers` - Maximum number of inliers for solvePnPRansac, stop if more inliers than this are found.
* `max_solvepnp_iter` - Maximum number of interations of the solvePnPRansac algorithm.
* `allowed_reprojection_err` - Maximum reprojection error allowed in solvePnPRansac algorithm.
* `stereo_vision_verbose` - True to output the messages of stereo matching process.
* `bucket_width` - Bucket width.
* `bucket_height` - Bucket height.
* `max_bucket_features` - Maximum number of features per bucket.

#### Topics ####
* `queue_size` - Indicate the maximum number of messages encued (typically between 2-6).
* `map_frame_id` - The map frame id (map by default).
* `base_link_frame_id` - The robot base link frame id.


Run the node
-------

You can run the node using the launch file located at launch/demo.launch:
```bash
roslaunch stereo_slam demo.launch
```


Online graph viewer
-------

The node provides a python script to visualize the results of the stereo_slam during the execution: scripts/graph_viewer.py:

```bash
usage: graph_viewer.py [-h]
	ground_truth_file visual_odometry_file
	graph_vertices_file graph_edges_file
```

The odometry file can be recorded directly from ros using:
```bash
rostopic echo -p /your_odometry_topic/odometry > odometry.txt
```

The ground truth file must have the same format than the odometry.


Post processing
-------

The node provides a python script to evaluate the results of the stereo_slam once the execution finishes: scripts/slam_evaluation.py:

```bash
usage: slam_evaluation.py [-h]
	ground_truth_file visual_odometry_file
	graph_vertices_file graph_edges_file
```

This script perform a set of operations in order to evaluate the performance of the stereo_slam algorithm:

1) Align all the curves (ground truth, visual_odometry and graph vertices) to the same origin.

2) Compute the average translation error.

3) Plot the error vs trajectory distance.


[link_ros]: http://www.ros.org/
[link_viso2]: http://wiki.ros.org/viso2_ros
[link_fovis]: http://wiki.ros.org/fovis_ros
[link_g2o]: http://wiki.ros.org/g2o
[link_yt_1]: http://www.youtube.com/watch?v=GXOhWmzSqUM
[link_yt_2]: http://www.youtube.com/watch?v=8NR6ono1SUI
