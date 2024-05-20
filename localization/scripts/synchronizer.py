#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3Stamped
from localization.msg import buffer

pub = rospy.Publisher("/imu_readings", buffer, queue_size=10)
rospy.init_node("sync_test")
r = rospy.Rate(10) #10hz
buffer_msg = buffer()

def sync_callback(imu_msg, mag_msg):
    # Create a buffer message
    buffer_msg.header.stamp = rospy.Time.now()
    # Assuming measurements is a list of 9 floats
    buffer_msg.measurements = imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z, imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z, mag_msg.magnetic_field.x*1e-6, mag_msg.magnetic_field.y*1e-6, mag_msg.magnetic_field.z*1e-6
    # Publish the synchronized data
    print(buffer_msg)
    pub.publish(buffer_msg)

while not rospy.is_shutdown():
    # Create a publisher for synchronized data
    

    # Create subscribers for IMU and magnetometer data
    imu_sub = message_filters.Subscriber("/imu/data", Imu)
    mag_sub = message_filters.Subscriber("/imu/magnetometer", MagneticField)

    # Synchronize the messages from subscribers
    sync = message_filters.ApproximateTimeSynchronizer(
        [imu_sub, mag_sub], queue_size=5, slop=0.03)

    sync.registerCallback(sync_callback)
    rospy.loginfo(buffer_msg)

    r.sleep()
    # rospy.spin()