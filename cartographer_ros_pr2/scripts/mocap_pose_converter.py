#!/usr/bin/env python

import rospy
import spinal.msg
import geometry_msgs.msg
import sensor_msgs.msg
import tf_conversions

class MocapConverter(object):
  def __init__(self):
    rospy.init_node("mocap_converter")
    self.raw_mocap_sub = rospy.Subscriber("/aerial_robot/pose", geometry_msgs.msg.PoseStamped, self.callback)
    self.mocap_pose_pub = rospy.Publisher("/mocap/pose", geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)
    self.pos_sigma = rospy.get_param("~pos_sigma", 0.001);
    self.att_sigma = rospy.get_param("~att_sigma", 0.005);

  def callback(self, data):

    pub_msg = geometry_msgs.msg.PoseWithCovarianceStamped()

    pub_msg.header = data.header
    pub_msg.header.frame_id = "imu_link"
    pub_msg.pose.pose = data.pose
    pub_msg.pose.covariance[0] = self.pos_sigma * self.pos_sigma
    pub_msg.pose.covariance[7] = self.pos_sigma * self.pos_sigma
    pub_msg.pose.covariance[14] = self.pos_sigma * self.pos_sigma
    pub_msg.pose.covariance[21] = self.att_sigma * self.att_sigma
    pub_msg.pose.covariance[28] = self.att_sigma * self.att_sigma
    pub_msg.pose.covariance[35] = self.att_sigma * self.att_sigma

    self.mocap_pose_pub.publish(pub_msg)

  def spin(self):
    rospy.spin()

if __name__ == "__main__":
  MocapConverter().spin()


