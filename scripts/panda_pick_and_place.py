#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Import required python modules
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from math import pi

###############################################################################

# Initialize the MoveIt commander and the rospy node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_pick_and_place", anonymous=True)

# Instantiate objects for accessing the robot and the environment scene
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
#print(robot.get_joint_names()) # displays all available robot joints

# The arm objects allows planning for a subgroup of the
# manipulator - in this case (only) the arm joints
arm_name = "panda_arm"
arm = moveit_commander.MoveGroupCommander(arm_name)

# Here we use a dedicated object "gripper" to access
# the finger joint of the panda robot
gripper = robot.get_joint('panda_finger_joint1')

# Save the current pose of the robot as initial_pose
# to which we will return later after executing the task
initial_pose = arm.get_current_pose()

###############################################################################
input('Initialized rospy and MoveIt commander. Press ENTER to continue.')
###############################################################################

# This function adds a box to the specified environment as 'scene', 
# using coordinates of the box in the frame specified by 'frame_id',
# giving the box the name 'name' with coordinates 'location,
# and dimensions as in 'dimensions'
def add_box(scene, frame_id, name, location, dimensions):    
    box_pose                 = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = frame_id
    box_pose.pose.position   = geometry_msgs.msg.Point(*location)
    
    box_name = name
    
    scene.add_box(box_name, box_pose, size=dimensions)

# This function creates the three relevant objects for the pick
# and place task, namely two tables and on object
def add_collision_objects(scene, frame_id):
    add_box(scene, frame_id, 'table1', location=[0.5, 0, 0.2], dimensions=[0.2, 0.4, 0.4])
    add_box(scene, frame_id, 'table2', location=[0.0, 0.5, 0.2], dimensions=[0.4, 0.2, 0.4])
    add_box(scene, frame_id, 'object', location=[0.5, 0.0, 0.5], dimensions=[0.02, 0.02, 0.2])


# Adds a set of objects (pre-defined above) to the our manipulation
# environment 'scene' and using coordinates in the panda base frame
add_collision_objects(scene, 'panda_link0')

###############################################################################
input('Added two tables and object to the scene. Press ENTER to continue.')
###############################################################################

# Define the position & orientation for the
# pick and place robot pose
PICK_POSITION    = [0.415, 0.0, 0.5]
PICK_ORIENTATION = [-pi/2, -pi/4, -pi/2]

PLACE_POSITION    = [0.0, 0.4, 0.5]
PLACE_ORIENTATION = [-pi/2, -pi/4, 0]

###############################################################################

# Create a pose 'pick_pose' and fill it with the position and 
# orientation data (where to pick the object) plus reference frame_id
pick_pose = geometry_msgs.msg.Pose()
pick_pose.position = geometry_msgs.msg.Point(*PICK_POSITION)
pick_pose.orientation = geometry_msgs.msg.Quaternion(*quaternion_from_euler(*PICK_ORIENTATION))


# Sets the target pose for the MoveIt planner
arm.set_pose_target(pick_pose)

# Performs the trajectory planning and moves the robot,
# returning a boolean indicating whether the planning and execution was successful.
success = arm.go(wait=True)

# Calling `stop()` ensures that there is no residual movement
arm.stop()

# It is always good to clear your targets after planning with poses.
arm.clear_pose_targets()

###############################################################################
input('Reached pose to pick the object. Press ENTER to continue.')
###############################################################################

# Close the gripper by moving it to 30% of its maximum joint range
# Note: the gripper should not yet touch the object -> collision detection
gripper.move(gripper.max_bound() * 0.30, True)

# Get all links belonging to the grasping_group of the Panda hand
grasping_group = "panda_hand"
touch_links = robot.get_link_names(group=grasping_group)

# Attach the object (virtually) to the end effector and thus
# disable collision detection between robot links and object
scene.attach_box(arm.get_end_effector_link(), 'object', touch_links=touch_links)

###############################################################################
input('Closed the gripper and attached the object. Press ENTER to continue.')
###############################################################################

# Repeat the same procedure as for picking now for placing the grasped object
# Create a pose 'place_pose', fill it with data
place_pose = geometry_msgs.msg.Pose()
place_pose.position = geometry_msgs.msg.Point(*PLACE_POSITION)
place_pose.orientation = geometry_msgs.msg.Quaternion(*quaternion_from_euler(*PLACE_ORIENTATION))

# Set pose target, plan & execute and then stop the robot arm
arm.set_pose_target(place_pose)
success = arm.go(wait=True)
arm.stop()
arm.clear_pose_targets()

###############################################################################
input('Reached pose to place the object. Press ENTER to continue.')
###############################################################################

# Open the gripper, i.e. move it to 90% of its joint limits
gripper.move(gripper.max_bound() * 0.9, True)

# Detach the object (virtually) from the robot joints and thus
# enable collision avoidance again between robot and object
scene.remove_attached_object(arm.get_end_effector_link(), name='object')

###############################################################################
input('Opened the gripper and detached the object. Press ENTER to continue.')
###############################################################################

# Move the robot back to its initial position
arm.set_pose_target(initial_pose)
success = arm.go(wait=True)
arm.stop()
arm.clear_pose_targets()

###############################################################################
print('Moved the robot to its initial pose. Exiting.')
###############################################################################
