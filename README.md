Fetchit GQCNN
=============

Grasp planner based on GQCNN and their policy demo.

Description
-----------

Modified version of gqcnn policy demo which takes in images from the fetch robot published on head_camera/depth_registered/image_raw and /head_camera/rgb/image_raw and publishes a grasp pose on /planned_grasp. The grasp pose encodes a postion to grasp at (with implict start width of 10 cm) and an angle to approach from as a quarternion.

Usage
---------
  * Start fetchit demo: ```roslaunch fetch_gazebo playground.launch```
  * Start movegroup: ```roslaunch fetch_moveit_config move_group.launch```
  * Source catkin workspace
  * Start mapping: ```roslaunch fetchit_world mapping.launch```
  * Move robot to place where it should lift an object
  * Clone this repository into your catkin_ws/src/ and cd into it
  * Get camera into postion with ```python src/top_down_view.py```
  * Run grasp planner ```python src/policy.py --config_filename=src/policy.yaml```; planned grasp will be published till the script is closed
