#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist,PoseStamped
from actionlib_msgs.msg import GoalID
class JoyNode:

    def __init__(self):
        rospy.init_node('joystick_node')

        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback, queue_size=10)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom_reset= rospy.Publisher("/initial_2d", PoseStamped, queue_size=10)
        self.nav_cancel= rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)
        self.start = 0
        self.lin_speed = 0.1
        self.ang_speed = 0.2
        
    def joy_callback(self, msg):
        # print(msg)
        if msg.buttons[0] == 1:
            self.start = 0
            if self.start == 1:
                rospy.loginfo("joystick working, current lin_speed: %f, ang_speed: %f", self.lin_speed, self.ang_speed)
            else:
                rospy.loginfo("joystick not working, current lin_speed: %f, ang_speed: %f", self.lin_speed, self.ang_speed)

        if msg.buttons[1] == 1:
            self.start = 1
            if self.start == 1:
                rospy.loginfo("joystick working, current lin_speed: %f, ang_speed: %f", self.lin_speed, self.ang_speed)
            else:
                rospy.loginfo("joystick not working, current lin_speed: %f, ang_speed: %f", self.lin_speed, self.ang_speed)

        if msg.buttons[4] == 1:
                self.lin_speed = self.lin_speed + 0.1
                if self.start == 1:
                    rospy.loginfo("joystick working, current lin_speed: %f, ang_speed: %f", self.lin_speed, self.ang_speed)
                else:
                    rospy.loginfo("joystick not working, current lin_speed: %f, ang_speed: %f", self.lin_speed, self.ang_speed)

        if msg.buttons[6] == 1:

                self.lin_speed = self.lin_speed - 0.1
                if self.start == 1:
                    rospy.loginfo("joystick working, current lin_speed: %f, ang_speed: %f", self.lin_speed, self.ang_speed)
                else:
                    rospy.loginfo("joystick not working, current lin_speed: %f, ang_speed: %f", self.lin_speed, self.ang_speed)

        if msg.buttons[5] == 1:

                self.ang_speed = self.ang_speed + 0.1
                if self.start == 1:
                    rospy.loginfo("joystick working, current lin_speed: %f, ang_speed: %f", self.lin_speed, self.ang_speed)
                else:
                    rospy.loginfo("joystick not working, current lin_speed: %f, ang_speed: %f", self.lin_speed, self.ang_speed)

        if msg.buttons[7] == 1:

                self.ang_speed = self.ang_speed - 0.1
                if self.start == 1:
                    rospy.loginfo("joystick working, current lin_speed: %f, ang_speed: %f", self.lin_speed, self.ang_speed)
                else:
                    rospy.loginfo("joystick not working, current lin_speed: %f, ang_speed: %f", self.lin_speed, self.ang_speed)

        if msg.buttons[3] == 1:
            rospy.loginfo('Everything resetted')
            self.start = 0
            self.lin_speed = 0.1
            self.ang_speed = 0.2
            if self.start == 1:
                rospy.loginfo("joystick working, current lin_speed: %f, ang_speed: %f", self.lin_speed, self.ang_speed)
            else:
                rospy.loginfo("joystick not working, current lin_speed: %f, ang_speed: %f", self.lin_speed, self.ang_speed)
                
        if msg.axes[5]==-1:
             self.nav_cancel_function()

        if msg.buttons[2] == 1:
             self.odom_resetting()

        if self.start == 1:
            if msg.axes[1] != 0.0 and msg.axes[3] != 0.0:
                self.send_velocity_command(msg.axes[1] * self.lin_speed, msg.axes[3] * self.ang_speed*-1 )
            else:
                self.send_velocity_command(msg.axes[1] * self.lin_speed, msg.axes[3] * self.ang_speed*-1)

    def odom_resetting(self):
         msg=PoseStamped()
         msg.pose.position.x=0.0
         msg.pose.position.y=0.0
         msg.pose.position.z=0.0
         msg.pose.orientation.x=0.0
         msg.pose.orientation.y=0.0
         msg.pose.orientation.z=0.0
         msg.pose.orientation.w=0.0
         self.odom_reset.publish(msg)
         print('odometry resetted')

    def nav_cancel_function(self):
        msg = GoalID()
        self.nav_cancel.publish(msg)
        print('navigation aborted')


    def send_velocity_command(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.vel_pub.publish(msg)


def main():
    joy_node = JoyNode()
    try:
        rospy.loginfo("Node started")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node ended")


if __name__ == "__main__":
    main()
