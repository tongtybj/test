#!/usr/bin/env python

import rospy
import spinal.msg
import geometry_msgs.msg
import sensor_msgs.msg
import tf_conversions
#from tf.transformations import *

class ImuConverter(object):
  def __init__(self):
    rospy.init_node("imu_converter")
    self.spinal_imu_sub = rospy.Subscriber("/imu", spinal.msg.Imu, self.callback)
    self.ros_imu_pub = rospy.Publisher("/ros_imu", sensor_msgs.msg.Imu, queue_size=10)
    self.prev_stamp = 0

  def callback(self, data):

    if self.prev_stamp > data.stamp.to_sec():
      rospy.logwarn("bad imu stamp")
      return

    pub_msg = sensor_msgs.msg.Imu()

    pub_msg.header.stamp = data.stamp
    pub_msg.header.frame_id = "imu_link"
    pub_msg.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(data.angles[0], data.angles[1], data.angles[0]))
    pub_msg.linear_acceleration = geometry_msgs.msg.Vector3(*data.acc_data)
    pub_msg.angular_velocity = geometry_msgs.msg.Vector3(*data.gyro_data)

    self.ros_imu_pub.publish(pub_msg)

    self.prev_stamp = data.stamp.to_sec()
  def spin(self):
    rospy.spin()

if __name__ == "__main__":
  ImuConverter().spin()


