Fetchit GQCNN
=============

Grasp planner based on GQCNN and their policy demo.

Usage
---------
  * Start fetchit demo: ```roslaunch fetch_gazebo playground.launch```
  * Start movegroup: ```roslaunch fetch_moveit_config move_group.launch```
  * Source catkin workspace
  * Start mapping: ```roslaunch fetchit_world mapping.launch```
  * Move robot to place where it should lift an object
  * Get camera into postion with ```python src/top_down_view.py```
  * Run grasp planner ```python src/policy.py --config_filename=src/policy.yaml```
