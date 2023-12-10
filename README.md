Stereo SLAM
=============

stereo_slam is a [ROS][link_ros] node to execute Simultaneous Localization And Mapping (SLAM) using only one stereo camera. The algorithm was designed and tested for underwater robotics. This node is based on the [G2O][link_g2o] library for graph optimization and uses the power of [libhaloc][link_libhaloc] to find loop closures between graph nodes. It uses a keyframe to multi-keyframe loop closing mechanism, based on keypoint clustering, to improve the SLAM corrections on feature-poor environments.

You can see it in action here:

[![Stereo-SLAM video](http://img.youtube.com/vi/S_eVKCKLFQc/0.jpg)](https://www.youtube.com/watch?v=S_eVKCKLFQc)

Specially designed for underwater scenarios:

[Video: ORB-SLAM vs Stereo SLAM (underwater)][link_yt_3]

More videos...

[Video: Stereo SLAM and 3D reconstruction ][link_yt_1] and
[Video: Stereo SLAM at UIB outdoor pond][link_yt_2]


Related paper
-------
[ICRA'16, nominated for the best student paper award][paper]

CITATION:
```bash
@INPROCEEDINGS{7487416,
  author={P. L. Negre and F. Bonin-Font and G. Oliver},
  booktitle={2016 IEEE International Conference on Robotics and Automation (ICRA)},
  title={Cluster-based loop closing detection for underwater slam in feature-poor regions},
  year={2016},
  pages={2589-2595},
  keywords={SLAM (robots);autonomous underwater vehicles;feature extraction;image matching;image registration;mobile robots;object detection;path planning;robot vision;AUV navigation;Balearic Islands;autonomous underwater vehicle;cluster-based loop closing detection;feature matching;feature-poor underwater environment;marine environments;sandbanks;seagrass;simultaneous localization and mapping;underwater SLAM;vision-based localization systems;visual features;visual information;visual keypoint clustering;visual registration;Cameras;Feature extraction;Image edge detection;Pipelines;Rocks;Simultaneous localization and mapping;Visualization},
  doi={10.1109/ICRA.2016.7487416},
  month={May}
}
```


Installation (Ubuntu + ROS)
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

* `refine` - Refine odometry (true/false).
* `distance_between_keyframes` - Minimum distance (m) between keyframes.
* `working_directory` - Directory where all output files will be stored.
* `feature_detector_selection` - Name of the feature detector to be used (ORB or SIFT).
* `lc_min_inliers` - Minimum number of inliers to close a loop.
* `lc_epipolar_thresh` - Maximum reprojection error allowed.
* `map_frame_name` - Frame of the slam output
* `lc_neighbors` - Number of neighbours to recover in order to increase the possibility of closing the loop.
* `lc_discard_window` - Window size of discarded vertices.
* `ransac_iterations` - Number of RANSAC iterations for the solvePnPRansac


Subscribed topics
-------

* `odom` - Input odometry (type nav_msgs::Odometry).
* `left_image_rect_color` - Left image in color and rectified (type sensor_msgs::Image).
* `right_image_rect_color`- Right image in color and rectified (type sensor_msgs::Image).
* `left_camera_info` - Extrinsic and intrinsic camera parameters (type sensor_msgs::CameraInfo).
* `right_camera_info`- Extrinsic and intrinsic camera parameters (type sensor_msgs::CameraInfo).


Published Topics
-------
* `/stereo_slam/odometry` - The odometry of the robot (type nav_msgs::Odometry).
* `/stereo_slam/graph_poses` - The updated graph poses (type stereo_slam::GraphPoses).
* `/stereo_slam/graph_camera_odometry` - Absolute pose of the camera (type nav_msgs::Odometry).
* `/stereo_slam/graph_robot_odometry` - Absolute pose of the vehicle (type nav_msgs::Odometry).
* `/stereo_slam/keyframes` - Number of inserted keyframes (type std_msgs::Int32).
* `/stereo_slam/keypoints_clustering` - Image containing the keypoint clusters (type sensor_msgs::Image).
* `/stereo_slam/stereo_matches_num` - Number of correspondences between the stereo pair (type std_msgs::Int32).
* `/stereo_slam/stereo_matches_img` - Correspondences between the stereo pair (type sensor_msgs::Image).
* `/stereo_slam/num_clusters` - Number of clusters (type std_msgs::Int32).
* `/stereo_slam/loop_closing_inliers_img` - Image of the loop closing correspondences. Correspondences are keyframe-to-multi-keyframe (type sensor_msgs::Image).
* `/stereo_slam/loop_closing_matches_num` - Number of matches of each loop closing (type std_msgs::Int32).
* `/stereo_slam/loop_closing_inliers_num` - Number of inliers of each loop closing (type std_msgs::Int32).
* `/stereo_slam/loop_closing_queue` - Number of keyframes waiting on the loop closing queue. Please monitor this topic, to check the real-time performance: if this number grows indefinitely it means that your system is not able to process all the keyframes, then, scale your images (type std_msgs::String).
* `/stereo_slam/loop_closings_num` - Number of loop closings found (type std_msgs::Int32).
* `/robot_0/stereo_slam/time_tracking` - The elapsed time of each iteration of the tracking thread (type stereo_slam::TimeTracking).
* `/robot_0/stereo_slam/time_graph` - The elapsed time of each iteration of the graph thread (type stereo_slam::TimeGraph).
* `/robot_0/stereo_slam/time_loop_closing` - The elapsed time of each iteration of the loop closing thread (type stereo_slam::TimeLoopClosing).
* `/robot_0/stereo_slam/sub_time_loop_closing` - The elapsed time in certain processes of the loop closing thread (type stereo_slam::SubTimeLoopClosing).


Run the node
-------

You can run the node using the following launch file (please, for a better performance scale your images if more than 960px width).

```bash
<launch>
  <arg name = "camera"        default = "/stereo"/>
  <arg name = "robot_name"  default = "robot_0"/>

  <!-- Run the stereo image proc -->
  <node ns = "$(arg camera)" pkg = "stereo_image_proc" type = "stereo_image_proc" name = "stereo_image_proc" />

  <node pkg = "viso2_ros" type = "stereo_odometer" name = "stereo_odometer">
    <remap from = "stereo" to = "$(arg camera)"/>
    <remap from = "image" to = "image_rect"/>
  </node>

  <node pkg="stereo_slam" type="localization" name="stereo_slam" output="screen" if = "$(eval enable_decimate_x1 == true)">
    <remap from = "odom"                        to = "stereo_odometer/odometry"/>
    <remap from = "left_camera_info"            to = "/$(arg camera)/left/camera_info"/>
    <remap from = "right_camera_info"           to = "/$(arg camera)/right/camera_info"/>
    <remap from = "left_image_rect_color"       to = "/$(arg camera)/left/image_rect_color"/>
    <remap from = "right_image_rect_color"      to = "/$(arg camera)/right/image_rect_color"/> 
    <param name = "refine"                      value = "false"/>
    <param name = "distance_between_keyframes"  value = "0.5"/>
    <param name = "feature_detector_selection"  value = "ORB"/>
    <param name = "lc_min_inliers"              value = "30"/>
    <param name = "lc_epipolar_thresh"          value = "1.0"/>
    <param name = "map_frame_name"              value = "/$(arg robot_name)/map"/>
    <param name = "lc_neighbors"                value = "5"/>
    <param name = "lc_discard_window"           value = "20"/>
    <param name = "ransac_iterations"           value = "150"/>
  </node>
</launch>
```


Saved data
-------
The node stores some data into the stereo_slam directory during the execution:
* `haloc` - A folder containing all the files needed for the libhaloc library, which is responsible for loop closing detection. You do not need this folder at all.
* `keyframes` - Stores the left stereo image for every keyframe (with the possibility of drawing the keypoint clustering over the image).
* `loop_closures` - Stores all the images published in the topic `/stereo_slam/loop_closing_inliers_img`.


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


[paper]: http://ieeexplore.ieee.org/document/7487416/
[link_ros]: http://www.ros.org/
[link_viso2]: http://wiki.ros.org/viso2_ros
[link_fovis]: http://wiki.ros.org/fovis_ros
[link_g2o]: http://wiki.ros.org/g2o
[link_libhaloc]: https://github.com/srv/libhaloc
[link_yt_1]: http://www.youtube.com/watch?v=GXOhWmzSqUM
[link_yt_2]: http://www.youtube.com/watch?v=8NR6ono1SUI
[link_yt_3]: https://www.youtube.com/watch?v=C4U8eaPzrLg
