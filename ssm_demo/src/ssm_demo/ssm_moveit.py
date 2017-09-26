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
from moveit_msgs.msg import MoveItErrorCodes
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
        ssm_state.ssmState.__init__(self, outcomes=['success', 'fail'], io_keys=['target'])

    def execution(self, ud):
        global get_pose
        #print ud.target
        pose_target = get_pose(ud.target)
        group.set_pose_target(pose_target)
        print "Move group %s to target pose " % (group.get_name())
        print pose_target.position.x, \
              pose_target.position.y, \
              pose_target.position.z, \
              pose_target.orientation.x, \
              pose_target.orientation.y, \
              pose_target.orientation.z, \
              pose_target.orientation.w

        plan = group.plan()
        if len(plan.joint_trajectory.points) == 0:
            print "Plan failed! Exiting state machine."
            return "fail"
        err = group.execute(plan, wait=True)
        print "execute response : "
        print err
        return "success"

class MoveNamed(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self, outcomes=['success', 'fail'], io_keys=['pose_name'])

    def execution(self, ud):
        group.set_named_target(ud.pose_name)
        plan = group.plan()
        if len(plan.joint_trajectory.points) == 0:
            print "Plan failed! Exiting state machine."
            return "fail"
        err = group.execute(plan, wait=True)
        return "success"

class MoveLine(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self, outcomes=['success', 'fail'], io_keys=['target'])

    def execution(self, ud):
        print "Move line"
        return "success"

#Only a parent state which calculates the approach pick and pick points and passes them to moveP2P
#Is the calculation a separate state? Better to call it PrePick or InitPick or something like that
class PickMove(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self, outcomes=['success', 'fail'], io_keys=['targets'])

    def execution(self, ud):
        print "performing pick"
        return "success"

class CalculatePickTrajectory(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self, outcomes=['success'], io_keys=['approach', 'pick', 'retract'])

    def execution(self, ud):
        print "calculating pick trajectory"
        return "success"

#Actually this is just a MoveP2P with gripper control -- no need for a separate state
class PlaceMove(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self, outcomes=['success', 'fail'], io_keys=['pose_name'])

    def execution(self, ud):
        print "performing place"
        return "success"

class GripperControl(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self, outcomes=['success'], io_keys=['action'])

    def execution(self, ud):
        print "Open / close gripper"
        return "success"

class InitRobot(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self,outcomes=['success'], io_keys=['group'])

    def execution(self, ud):
        global group
        group = moveit_commander.MoveGroupCommander(ud.group)
        #print group.get_end_effector_link()
        return "success"


