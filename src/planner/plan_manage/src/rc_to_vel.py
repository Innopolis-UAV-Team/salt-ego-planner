#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import RCIn, State
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
import math
import utils

class RCToVelocityConverter:
    def __init__(self):
        rospy.init_node('rc_to_velocity_converter', anonymous=True)

        self.horizontal_speed_koef = rospy.get_param('/rc_to_velocity_converter/horizontal_speed', 6.0)
        self.vertical_speed_koef = rospy.get_param('/rc_to_velocity_converter/vertical_speed', 1.5)
        self.angle_speed_koef = rospy.get_param('/rc_to_velocity_converter/angle_speed', 2.0)

        rospy.Subscriber('/mavros/rc/in', RCIn, self.rc_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_pose_callback)
        rospy.Subscriber('/mavros/state', State, self.status_callback)

        self.velocity_publisher = rospy.Publisher('/rc_to_velocity_converter/velocity_from_rc', TwistStamped, queue_size=10)

        # current position from topic
        self.local_pose = PoseStamped()
        # rc control from topic
        self.rc_control = Twist()

        self.rc_active_any = False
        self.rc_active_thr = False
        self.is_offboard = False
        
        self.rate = rospy.Rate(40)

    def rc_callback(self, msg):
        channels = msg.channels
        
        normalized_channels = [(i - 1500) / 500.0 for i in channels]  # 1500 - center, 500 - range
        roll, throttle, pitch, yaw = normalized_channels[:4]
        
        self.rc_control.linear.x = pitch
        self.rc_control.linear.y = -roll
        self.rc_control.linear.z = -throttle
        self.rc_control.angular.z = yaw * -10

        eps = 10e-4
        self.rc_active_any = abs(roll) >= eps or abs(pitch) >= eps or abs(throttle) >= eps or abs(yaw) >= eps
        self.rc_active_thr = abs(throttle) >= eps

    def local_pose_callback(self, msg):
        if self.is_offboard:
            if self.rc_active_any:
                if not self.rc_active_thr:
                    msg.pose.position.z = self.local_pose.pose.position.z
                self.local_pose = msg
        else:
            self.local_pose = msg

    def status_callback(self, msg):
        self.is_offboard = msg.mode == "OFFBOARD"

    def generate_desired_velocity(self):
        desired_velocity = TwistStamped()
        desired_velocity.header = self.local_pose.header
        yaw = utils.get_yaw_from_quaternion(self.local_pose.pose.orientation)
        desired_velocity.twist.linear.x = (self.rc_control.linear.x * math.cos(yaw) - self.rc_control.linear.y * math.sin(yaw)) * self.horizontal_speed_koef
        desired_velocity.twist.linear.y = (self.rc_control.linear.x * math.sin(yaw) + self.rc_control.linear.y * math.cos(yaw)) * self.horizontal_speed_koef
        desired_velocity.twist.linear.z = self.rc_control.linear.z * self.vertical_speed_koef
        desired_velocity.twist.angular.z = self.rc_control.angular.z * self.angle_speed_koef
        return desired_velocity

    def run(self):
        while not rospy.is_shutdown():
            desired_velocity = self.generate_desired_velocity()
            self.velocity_publisher.publish(desired_velocity)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        converter = RCToVelocityConverter()
        converter.run()
    except rospy.ROSInterruptException:
        pass
