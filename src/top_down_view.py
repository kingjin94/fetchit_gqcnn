import rospy
rospy.init_node("top_down_view")

## Allows to rotate the head
# Needs movegroup running: "roslaunch fetch_moveit_config move_group.launch"
print("Moving Camera")
import copy
import actionlib
import rospy

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()
    
# rospy.init_node("demo")
head_action = PointHeadClient()
head_action.look_at(0, 0, 0.0, "base_link")  # x = to the front, z = up (0 = floor), y = left
print("Camera moved")

## Move torso up
print("Moving Torso")
import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface

# Note: fetch_moveit_config move_group.launch must be running
# Safety!: Do NOT run this script near people or objects.
# Safety!: There is NO perception.
#          The ONLY objects the collision detection software is aware
#          of are itself & the floor.

# rospy.init_node("move_torso_up")

# Create move group interface for a fetch robot
move_group = MoveGroupInterface("arm_with_torso", "base_link")

# Define ground plane
# This creates objects in the planning scene that mimic the ground
# If these were not in place gripper could hit the ground
planning_scene = PlanningSceneInterface("base_link")
planning_scene.removeCollisionObject("my_front_ground")
planning_scene.removeCollisionObject("my_back_ground")
planning_scene.removeCollisionObject("my_right_ground")
planning_scene.removeCollisionObject("my_left_ground")
planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

# TF joint names
joint_names = ["torso_lift_joint", "shoulder_pan_joint",
               "shoulder_lift_joint", "upperarm_roll_joint",
               "elbow_flex_joint", "forearm_roll_joint",
               "wrist_flex_joint", "wrist_roll_joint"]
# Lists of joint angles in the same order as in joint_names
# disco_poses = [[4.0, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]] # Max torso and keep arm tugged away
disco_poses = [[4.0, 2.3, 0, 0, 0, 0.0, 0, 0.0]]  # Max torso, arm streched out to be out of the view

for pose in disco_poses:
    if rospy.is_shutdown():
        break

    # Plans the joints in joint_names to angles in pose
    move_group.moveToJointPosition(joint_names, pose, wait=False)

    # Since we passed in wait=False above we need to wait here
    move_group.get_move_action().wait_for_result()
    result = move_group.get_move_action().get_result()

    if result:
        # Checking the MoveItErrorCode
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Extended Torso.")
        else:
            # If you get to this point please search for:
            # moveit_msgs/MoveItErrorCodes.msg
            rospy.logerr("Arm goal in state: %s",
                         move_group.get_move_action().get_state())
    else:
        rospy.logerr("MoveIt! failure no result returned.")

# This stops all arm movement goals
# It should be called when a program is exiting so movement stops
move_group.get_move_action().cancel_all_goals()