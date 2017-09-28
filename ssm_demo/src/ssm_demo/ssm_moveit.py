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
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import ExecuteTrajectoryActionResult 
from moveit_msgs.msg import MoveItErrorCodes
import ast
import copy
import actionlib

group = None
tf_listener = None

#convert string to list
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

#Given a list as string (user data), convert it into Pose msg
#Some states might directly return the Pose, so no need
#for conversion
def get_pose(str):
    global get_list
    try:
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
    except:
        pose_target = str
    return pose_target

#C++ code I used to transform pose, as reference
'''
tf::Stamped<tf::Pose> p1, p2;

p1.frame_id_ = "panel";
p1.setOrigin(tf::Vector3(p.position.x,
                         p.position.y,
                         p.position.z));
std::cout << "panel point : " << p1.getOrigin().x() << " " << p1.getOrigin().y() << " " << p1.getOrigin().z() << std::endl;

tf::Quaternion q;
q.setRPY(0, 0, 0);
p1.setRotation(q);

tf_listener->transformPose("ur5_base_link", p1, p2);
std::cout << "wrench in base_link frame : " << p2.getOrigin().x() << " " <<
             p2.getOrigin().y() << " " <<
             p2.getOrigin().z() << std::endl;

geometry_msgs::Pose pose;
pose.position.x = p2.getOrigin().x();
pose.position.y = p2.getOrigin().y();
pose.position.z = p2.getOrigin().z();
}
'''

#Transform pose from ref_frame to target_frame
def get_pose_frame(pose, ref_frame, target_frame):
    p1 = PoseStamped()
    p2 = PoseStamped()

    p1.pose.position.x = pose.position.x
    p1.pose.position.y = pose.position.y
    p1.pose.position.z = pose.position.z
    p1.pose.orientation.x = pose.orientation.x
    p1.pose.orientation.y = pose.orientation.y
    p1.pose.orientation.z = pose.orientation.z
    p1.pose.orientation.w = pose.orientation.w

    p1.header.frame_id = ref_frame

    tf_listener.waitForTransform(ref_frame, target_frame, rospy.Time(0), rospy.Duration(4.0))
    p2 = tf_listener.transformPose(target_frame, p1)

    pose_target = Pose()
    pose_target.position.x = p1.pose.position.x
    pose_target.position.y = p1.pose.position.y
    pose_target.position.z = p1.pose.position.z
    pose_target.orientation.x = p1.pose.orientation.x
    pose_target.orientation.y = p1.pose.orientation.y
    pose_target.orientation.z = p1.pose.orientation.z
    pose_target.orientation.w = p1.pose.orientation.w
    return pose_target

class MoveP2P(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self, outcomes=['success', 'fail'], io_keys=['target'])

    def execution(self, ud):
        global get_pose
        print ud.target
        pose_target = get_pose(ud.target)
        group.set_pose_target(pose_target)
        print "Move group %s to target pose " % (group.get_name())
        print pose_target
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

#The tool_pin_rotated is set as the tip_link for the IK plugin in SRDF
#So no need to rotate the end-effector pose by 180
#Currently not trying to limit the rotation within 120. That's for later
#'targets' should be set by the scanner's Next service. For now settig manually
class CalculatePickTrajectory(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self, outcomes=['success'], io_keys=['traj_in', 'world_gripper_pose', 'traj_out_1', 'approach', 'pick', 'retract'])

    def execution(self, ud):
        global get_pose
        global get_pose_frame
        print "calculating pick trajectory"
        ud.approach = get_pose_frame(get_pose(ud.traj_in), "bin", "base_link")
        ud.pick     = get_pose_frame(get_pose(ud.world_gripper_pose), "bin", "base_link")
        ud.retract  = get_pose_frame(get_pose(ud.traj_out_1), "bin", "base_link")
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
        print "gripper "
        print ud.action
        return "success"

class InitRobot(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self,outcomes=['success'], io_keys=['group'])

    def execution(self, ud):
        global group
        global tf_listener
        group = moveit_commander.MoveGroupCommander(ud.group)
        tf_listener = tf.TransformListener()

        #print group.get_end_effector_link()
        return "success"


