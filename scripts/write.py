#!/usr/bin/env python

"""
Reads a cartesian trajectory on /write_traj, and tries to execute it on a robot.

Requires:
    - a running MoveIt! setup for the robot.
        -> a ROS service called 'compute_ik' is expected to be available

The script tries as much as possible to be robot-agnostic. The only requirement is
that a SRDF planning group called "right_arm_and_torso" exists.

"""

import logging
logger = logging.getLogger("write." + __name__)
logger.setLevel(logging.DEBUG)

handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter('[%(levelname)s] %(name)s -> %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

import sys
import math

import rospy
import tf

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState, DisplayRobotState

from geometry_msgs.msg import Transform, PoseStamped
from moveit_msgs.msg import MultiDOFJointTrajectory


TRAJ_TOPIC = "/write_traj"
ENDEFFECTOR = "/R_WR"
WRITING_FRAME = "/pen_tip"

rospy.wait_for_service('compute_ik')
compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

pub = rospy.Publisher('display_robot_state', DisplayRobotState)
pub_ik_target = rospy.Publisher('ik_target', PoseStamped)


rs = DisplayRobotState()

rospy.init_node("love_letters_receiver")


tl = tf.TransformListener()

def get_ik(target, group = "right_arm_and_torso"):
    """

    :param target:  a PoseStamped give the desired position of the endeffector.
    """

    
    service_request = PositionIKRequest()
    service_request.group_name = group
    service_request.ik_link_name = ENDEFFECTOR
    service_request.pose_stamped = target
    service_request.timeout.secs= 0.005
    service_request.avoid_collisions = False

    try:
        resp = compute_ik(ik_request = service_request)
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


transformation_to_writing_frame = None
#transformation_to_writing_frame = Transform()
#
#if WRITING_FRAME:
#    tl.waitForTransform(ENDEFFECTOR,  WRITING_FRAME, rospy.Time(), rospy.Duration(1))
#    t = tl.getLatestCommonTime(ENDEFFECTOR, WRITING_FRAME)
#    transformation_to_writing_frame = tl.lookupTransform(ENDEFFECTOR, WRITING_FRAME, t)
#
def on_traj(traj):
    logger.info("Got a new traj to follow! cool!")

    first = True

    target = PoseStamped()

    target_frame = traj.header.frame_id

    target.header.frame_id = target_frame

    offset = [0] * 3

    if WRITING_FRAME:
        tl.waitForTransform(target_frame, ENDEFFECTOR, rospy.Time(), rospy.Duration(1))
        t = tl.getLatestCommonTime(ENDEFFECTOR, target_frame)
        p1, q1 = tl.lookupTransform(target_frame, ENDEFFECTOR, t)
        p2, q2 = tl.lookupTransform(target_frame, WRITING_FRAME, t)

        offset = [a1-a2 for a1,a2 in zip(p1, p2)]

    # Get the transformation between the end-effector and the target frame
    # to use the current orientation of the end-effector (do not try to follow the
    # paper)
    tl.waitForTransform(ENDEFFECTOR,  target_frame, rospy.Time(), rospy.Duration(1))
    t = tl.getLatestCommonTime(ENDEFFECTOR, target_frame)
    toto, quaternion = tl.lookupTransform(target_frame, ENDEFFECTOR, t)

    qx,qy,qz,qw = quaternion
    target.pose.orientation.x = qx
    target.pose.orientation.y = qy
    target.pose.orientation.z = qz
    target.pose.orientation.w = qw


    logger.info("%s points in the traj."% len(traj.points))

    missed = 0
    for p in traj.points:
        target.pose.position = p.transforms[0].translation
        target.pose.position.x += offset[0]
        target.pose.position.y += offset[1]
        target.pose.position.z += offset[2]

        pub_ik_target.publish(target)
        res = get_ik(target)
        if res.error_code.val != 1:
            #logger.error("Unreachable point %s (error: %s)" % (target, res.error_code))
            logger.error("Unreachable point!")
            missed += 1
        else:
            rs.state = res.solution
            if first:
                print(", ".join(str(x) for x in rs.state.joint_state.name))
                first = False

            print(", ".join(str(x) for x in rs.state.joint_state.position))
            pub.publish(rs)
            rospy.sleep(0.01)

    logger.info("%s points were not reachable by the robot." % missed)

logger.info("Waiting for incoming trajectories on %s" % TRAJ_TOPIC)
pub_traj = rospy.Subscriber(TRAJ_TOPIC, MultiDOFJointTrajectory, on_traj)
rospy.spin()
