#!/usr/bin/env python3

# subscribes to cmd_vel and publisehes Odometry

import rospy
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
from tf.broadcaster import TransformBroadcaster
import math

def cmd_vel_callback(msg):
    global vx, vy, vth
    vx = msg.linear.x
    vy = msg.linear.y
    vth = msg.angular.z

def odometry_publisher():
    rospy.init_node("odometry_publisher")

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = TransformBroadcaster()

    global x, y, th, vx, vy, vth
    x, y, th = 0.0, 0.0, 0.0
    vx, vy, vth = 0.0, 0.0, 0.0
    last_time = rospy.Time.now()  # Initialize last_time here

    rate = rospy.Rate(10)  # 10 Hz

    rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # Compute odometry
       # Compute odometry
        dt = (current_time - last_time).to_sec()
        # print(dt, current_time)
        delta_x = (vx * math.cos(th) - vy * math.sin(th)) * dt
        delta_y = (vx * math.sin(th) + vy * math.cos(th)) * dt
        delta_th = vth * dt

        x += delta_x
        y += delta_y
        th += delta_th
        print(x,y,th)

        # Publish the transform over tf
        odom_quat = Quaternion(*quaternion_from_euler(0, 0, th))
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = odom_quat.x
        t.transform.rotation.y = odom_quat.y
        t.transform.rotation.z = odom_quat.z
        t.transform.rotation.w = odom_quat.w

        # Send the transform
        # odom_broadcaster.sendTransform(t)
        odom_broadcaster.sendTransform(
        translation=(x, y, 0.0),
        rotation=(odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w),
        time=current_time,
        child="base_link",
        parent="odom"
)

        # Publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # Set the position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat.x
        odom.pose.pose.orientation.y = odom_quat.y
        odom.pose.pose.orientation.z = odom_quat.z
        odom.pose.pose.orientation.w = odom_quat.w

        # Set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        # Publish the message
        odom_pub.publish(odom)

        last_time = current_time
        rate.sleep()

if __name__ == "__main__":
    try:
        odometry_publisher()
    except rospy.ROSInterruptException:
        pass

