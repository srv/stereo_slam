Stereo SLAM
=============

stereo_slam is a [ROS][link_ros] node to execute Simultaneous Localization And Mapping (SLAM) using only one stereo camera. The algorithm was designed and tested for underwater robotics. This node is based on the [G2O][link_g2o] library for graph optimization and uses the power of [libhaloc][link_libhaloc] to find loop closures between graph nodes. It uses a keyframe to multi-keyframe loop closing mechanism, based on keypoint clustering, to improve the SLAM corrections on feature-poor environments.

You can see it in action here:

[![Alt text for your video](http://img.youtube.com/vi/h3FfXafuOvE/0.jpg)](http://www.youtube.com/watch?v=h3FfXafuOvE)

Specially designed for underwater scenarios:

[Video: ORB-SLAM vs Stereo SLAM (underwater)][link_yt_3]

More videos...

[Video: Stereo SLAM and 3D reconstruction ][link_yt_1] and
[Video: Stereo SLAM at UIB outdoor pond][link_yt_2]

Installation (Ubuntu + ROS Indigo)
-------

1) Install the dependencies
```bash
sudo apt-get install ros-<your ros distro>-libg2o

sudo apt-get install libceres-dev
```

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

* `odom_topic` - Visual odometry topic (type nav_msgs::Odometry).
* `camera_topic` - The namespace of your stereo camera.

Other (hard-coded) parameters

* `include/constants.h` - Contains the set of node parameters. Default parameters should work.


Run the node
-------

You can run the node using the following launch file (please, for a better performance scale your images if more than 960px width).

```bash
<launch>
  <arg name="camera" default="/stereo"/>

  <!-- Run the stereo image proc -->
  <node ns="$(arg camera)" pkg="stereo_image_proc" type="stereo_image_proc" name="stereo_image_proc" />

  <node pkg="viso2_ros" type="stereo_odometer" name="stereo_odometer">
    <remap from="stereo" to="$(arg camera)"/>
    <remap from="image" to="image_rect"/>
  </node>

  <node pkg="stereo_slam" type="localization" name="stereo_slam" output="screen">
    <param name="odom_topic" value="/stereo_odometer/odometry"/>
    <param name="camera_topic" value="$(arg camera)"/>
  </node>
</launch>
```

Published Topics
-------
* `/stereo_slam/odometry` - The vehicle pose (type nav_msgs::Odometry).
* `/stereo_slam/graph_poses` - The updated graph poses (type stereo_slam::GraphPoses).
* `/stereo_slam/keyframes` - Number of inserted keyframes (type std_msgs::String).
* `/stereo_slam/keypoints_clustering` - Image containing the keypoint clusters (type sensor_msgs::Image).
* `/stereo_slam/loop_closing_matchings` - Image of the loop closing correspondences. Correspondences are keyframe-to-multi-keyframe (type sensor_msgs::Image).
* `/stereo_slam/loop_closing_queue` - Number of keyframes waiting on the loop closing queue. Please monitor this topic, to check the real-time performance: if this number grows indefinitely it means that your system is not able to process all the keyframes, then, scale your images. (type std_msgs::String).
* `/stereo_slam/loop_closings` - Number of loop closings found (type std_msgs::String).
* `/stereo_slam/pointcloud` - The pointcloud for every keyframe (type sensor_msgs::PointCloud2).
* `/stereo_slam/tracking_overlap` - Image containing a representation of the traking overlap. Used to decide when to insert a new keyframe into the graph (type sensor_msgs::Image).
* `/stereo_slam/camera_params` - The optimized (calibrated) camera parameters after every loop closure (type stereo_slam::CameraParams).


Saved data
-------
The node stores some data into the stereo_slam directory during the execution:
* `haloc` - A folder containing all the files needed for the libhaloc library, which is responsible for loop closing detection. You do not need this folder at all.
* `keyframes` - Stores the left stereo image for every keyframe (with the possibility of drawing the keypoint clustering over the image).
* `loop_closures` - Stores all the images published in the topic `/stereo_slam/loop_closing_matchings`.
* `pointclouds` - Stores all the pointclouds published in the topic `/stereo_slam/pointcloud`.


Online graph viewer
-------

The node provides a python script to visualize the results of the stereo_slam during the execution: scripts/graph_viewer.py:

```bash
usage: 	graph_viewer.py [-h]
	ground_truth_file visual_odometry_file
	graph_vertices_file graph_edges_file
```

or (automatically detect the graph files):

```bash
roscd stereo_slam
./scripts/graph_viewer.py
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
[link_yt_3]: https://www.youtube.com/watch?v=C4U8eaPzrLg
