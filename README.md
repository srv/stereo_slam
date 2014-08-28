Stereo SLAM
=============

stereo_slam is a [ROS][link_ros] node to execute Simultaneous Localization And Mapping (SLAM) using only one stereo camera. The algorithm was designed and tested for underwater robotics. This node is based on the [G2O][link_g2o] library for graph optimization and uses the power of [libhaloc][link_libhaloc] to find loop closures between graph nodes. The workflow of the stereo_slam node is as follows:

![alt tag](https://raw.github.com/srv/stereo_slam/hydro/resources/flowchart.png)

You can see it in action here:

[![Alt text for your video](http://img.youtube.com/vi/h3FfXafuOvE/0.jpg)](http://www.youtube.com/watch?v=h3FfXafuOvE)

More videos...
[Stereo SLAM and 3D reconstruction ][link_yt_1] and 
[Stereo SLAM at UIB outdoor pond][link_yt_2]

Installation (Ubuntu + ROS fuerte/hydro)
-------

1) Install the dependencies
```bash
sudo apt-get install ros-<your ros distro>-libg2o
```

Install libhaloc as specified here: [libhaloc][link_libhaloc].

You also need to setup a stereo visual odometer (e.g. [viso2][link_viso2] or [fovis][link_fovis]).

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
* `min_matches` - Minimun number of descriptor matches to consider a matching as possible loop closure. If you don't have loop closings, try to decrese this parameter (minimum value = 8).
* `min_inliers` - Minimum number of inliers between loop closing candidates. If you don't have loop closings, try to decrese this parameter (minimum value = 7).


### Other parameters (do not touch by default) ###

#### G2O Library ####
* `g2o_algorithm` - Set to 0 for LinearSlam Solver with gauss-newton. Set to 1 for LinearSlam Solver with Levenberg (Default 1).
* `g2o_opt_max_iter` - Maximum number of g2o alogirthm iterations (typically between 10-50)

#### Graph ####
* `min_neighbour` - Minimum number of neighbours to considerate for loop closing.

#### Stereo vision ####
* `desc_type` - Can be SIFT or SURF (SIFT by default).
* `desc_thresh` - Descriptor threshold (for SIFT typically between 0.8-0.9).

#### Topics ####
* `pose_frame_id` - Frame name where pose will be published.
* `pose_child_frame_id` - Child frame name of the pose.


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
usage: 	graph_viewer.py [-h]
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
[link_libhaloc]: https://github.com/srv/libhaloc
[link_yt_1]: http://www.youtube.com/watch?v=GXOhWmzSqUM
[link_yt_2]: http://www.youtube.com/watch?v=8NR6ono1SUI
