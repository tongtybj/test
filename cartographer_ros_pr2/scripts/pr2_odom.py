#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf

class Odom(object):
  def __init__(self):
      rospy.init_node("odom")
      self.odom_sub = rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.callback)
      self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

  def callback(self, data):

      pub_msg = Odometry()
      pub_msg.header = data.header
      pub_msg.child_frame_id = "base_footprint"
      pub_msg.pose = data.pose
      self.odom_pub.publish(pub_msg)

  def spin(self):
      rospy.spin()

if __name__ == "__main__":
  Odom().spin()


