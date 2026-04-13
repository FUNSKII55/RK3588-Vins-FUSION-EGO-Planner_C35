#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu

pub = None

def cb(msg):
    out = Imu()
    out.header = msg.header
    out.header.frame_id = "imu_link"

    #optical to FLU
    out.linear_acceleration.x =  msg.linear_acceleration.z
    out.linear_acceleration.y = -msg.linear_acceleration.x
    out.linear_acceleration.z = -msg.linear_acceleration.y

    out.angular_velocity.x =  msg.angular_velocity.z
    out.angular_velocity.y = -msg.angular_velocity.x
    out.angular_velocity.z = -msg.angular_velocity.y

    pub.publish(out)

if __name__ == '__main__':
    rospy.init_node('imu_transform')
    pub = rospy.Publisher('/imu/data_flu', Imu, queue_size=10)
    rospy.Subscriber('/camera/imu', Imu, cb)
    rospy.spin()
