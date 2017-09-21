#!/usr/bin/env python

import sys
import rospy
import tf
import numpy as np
from airbus_ssm_core import ssm_state
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from gazebo_msgs.srv import SpawnModel
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from moveit_msgs.msg import ExecuteTrajectoryActionResult 
import ast
import copy
import actionlib

group = None

def get_list(str):
    list = ast.literal_eval(str)
    return list

def eulerToQuaternion(angles):
        roll = np.deg2rad(angles[3])
        pitch = np.deg2rad(angles[4])
        yaw = np.deg2rad(angles[5])
        quaternion_tf = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        quaternion = [quaternion_tf[0],quaternion_tf[1],quaternion_tf[2],quaternion_tf[3]]
        return quaternion

def get_pose(str):
    global get_list
    pose = get_list(str)
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = pose[0]
    pose_target.position.y = pose[1]
    pose_target.position.z = pose[2]

    quat = eulerToQuaternion(pose)
    pose_target.orientation.x = quat[0]
    pose_target.orientation.y = quat[1]
    pose_target.orientation.z = quat[2]
    pose_target.orientation.w = quat[3]
    return pose_target


class MoveP2P(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self, outcomes=['success'], io_keys=['target'])

    def execution(self, ud):
        global get_pose
        print ud.target
        pose_target = get_pose(ud.target)
        group.set_pose_target(pose_target)
        #print "Move group %s to target pose " % (group.get_end_effector_link())
        print pose_target.position.x, \
              pose_target.position.y, \
              pose_target.position.z, \
              pose_target.orientation.x, \
              pose_target.orientation.y, \
              pose_target.orientation.z, \
              pose_target.orientation.w

        plan = group.plan()
        group.execute(plan, wait=True)
        return "success"

class InitRobot(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self,outcomes=['success'], io_keys=['group'])

    def execution(self, ud):
        global group
        group = moveit_commander.MoveGroupCommander(ud.group)
        print group.get_end_effector_link()
        return "success"

#class HomeRobot(ssm_state.ssmState):
#    def __init__(self):
#        ssm_state.ssmState.__init__(self,outcomes=['success'], io_keys=['target'])
#
#    def execution(self, ud):
#        global eulerToQuaternion
#        ud.target = [0.3, 0, 0.8, 0, 90, 0]
#
#        return "success"
