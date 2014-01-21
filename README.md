Stereo SLAM
=============

stereo_slam is a [ROS][link_ros] node to execute Simultaneous Localization And Mapping using one stereo camera. This node is based on the [G2O][link_g2o] library for graph optimization and uses the power of PostgreSQL to store the image keypoints and descriptors. The workflow of the stereo_slam node is as follows:

![alt tag](https://raw.github.com/srv/stereo_slam/fuerte/resources/flowchart.png)


Installation (Ubuntu + ROS fuerte)
-------

1) Install the dependencies
	- sudo apt-get install ros-fuerte-libg2o
	- sudo apt-get install ros-fuerte-sql-database
	- sudo apt-get install postgresql pgadmin3

	You also need to setup a stereo visual odometer (e.g. [viso2][link_viso2] or [fovis][link_fovis]).

2) Configure the PostgreSQL database. This creates the user postgres with password postgres. You can setup a different user and password but then you must specify this parameters to the stereo_slam ROS node.
	- sudo -u postgres psql postgres
	- \password postgres

3) Download the code, save it to your ROS workspace enviroment and compile. 
	- roscd
	- git clone https://github.com/srv/stereo_slam.git
	- cd stereo_slam
	- rosmake


Parameters
-------



Run the node
-------


Real time graph viewer
-------


Post processing
-------



[link_ros]: http://www.ros.org/
[link_viso2]: http://wiki.ros.org/viso2_ros
[link_fovis]: http://wiki.ros.org/fovis_ros
[link_g2o]: http://wiki.ros.org/g2o