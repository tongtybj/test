#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import *

class ImuAddGravity(object):
  def __init__(self):
      rospy.init_node("imu_converter")
      self.raw_imu_sub = rospy.Subscriber("/torso_lift_imu/data", Imu, self.callback)
      self.imu_with_gravity_pub = rospy.Publisher("/imu", Imu, queue_size=10)
      self.gravity = [0, 0, 9.806, 1]

  def callback(self, data):
      pub_msg = data
      q = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
      g_b = numpy.dot(quaternion_matrix(quaternion_inverse(q)), self.gravity)
      pub_msg.linear_acceleration.x = data.linear_acceleration.x + g_b[0]
      pub_msg.linear_acceleration.y = data.linear_acceleration.y + g_b[1]
      pub_msg.linear_acceleration.z = data.linear_acceleration.z + g_b[2]

      self.imu_with_gravity_pub.publish(pub_msg)

  def spin(self):
      rospy.spin()

if __name__ == "__main__":
  ImuAddGravity().spin()


