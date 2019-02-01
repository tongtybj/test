#!/usr/bin/env python

import rospy
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pr2_msgs.msg import PeriodicCmd
from pr2_msgs.srv import SetPeriodicCmd
if __name__ == "__main__":
    rospy.init_node("tuck_arms")

    rospy.wait_for_service('/laser_tilt_controller/set_periodic_cmd')
    try:
        enable_tilt_laser = rospy.ServiceProxy('/laser_tilt_controller/set_periodic_cmd', SetPeriodicCmd)
        cmd = PeriodicCmd()
        #cmd.header.stamp = 0
        cmd.profile = "linear"
        cmd.period = 1
        cmd.amplitude = 1
        cmd.offset = 0
        resp = enable_tilt_laser(cmd)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    larm_pub = rospy.Publisher('/l_arm_controller/command', JointTrajectory, queue_size=1)
    ltraj = JointTrajectory()
    ltraj.joint_names  = [
        'l_shoulder_pan_joint',
        'l_shoulder_lift_joint',
        'l_upper_arm_roll_joint',
        'l_elbow_flex_joint',
        'l_forearm_roll_joint',
        'l_wrist_flex_joint',
        'l_wrist_roll_joint'
    ]

    ltraj.points.append(JointTrajectoryPoint())
    ltraj.points[0].positions = [
        1.5,
        1.4,
        0.0,
        -2.5,
        0.0,
        0.0,
        0.0,
    ]
    ltraj.points[0].time_from_start = rospy.Duration(1)

    rarm_pub = rospy.Publisher('/r_arm_controller/command', JointTrajectory, queue_size=1)
    rtraj = JointTrajectory()
    rtraj.joint_names  = [
        'r_shoulder_pan_joint',
        'r_shoulder_lift_joint',
        'r_upper_arm_roll_joint',
        'r_elbow_flex_joint',
        'r_forearm_roll_joint',
        'r_wrist_flex_joint',
        'r_wrist_roll_joint'
    ]

    rtraj.points.append(JointTrajectoryPoint())
    rtraj.points[0].positions = [
        -1.5,
        1.4,
        0.0,
        -2.5,
        0.0,
        0.0,
        0.0,
    ]
    rtraj.points[0].time_from_start = rospy.Duration(1)

    time.sleep(1)
    larm_pub.publish(ltraj)
    rarm_pub.publish(rtraj)
