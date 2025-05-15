#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
import utils
import random

class VelocityToPositionConverter:
    def __init__(self):
        rospy.init_node('vel_to_pose_converter', anonymous=True)

        rospy.Subscriber('/rc_to_velocity_converter/velocity_from_rc', TwistStamped, self.velocity_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_pose_callback)

        self.position_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # velocity from topic
        self.desired_velocity = TwistStamped()
        # current position from topic
        self.local_pose = PoseStamped()
        
        self.rate = rospy.Rate(40)

    def local_pose_callback(self, msg):
        self.local_pose = msg

    def velocity_callback(self, msg):
        self.desired_velocity = msg

    def generate_desired_position(self, control_velocity):
        desired_pose = PoseStamped()
        desired_pose.pose.position.x = 0 + random.uniform(-0.01, 0.01) #self.local_pose.pose.position.x + self.desired_velocity.twist.linear.x #+ self.repulsion_vector.x
        desired_pose.pose.position.y = 0 + random.uniform(-0.01, 0.01) #self.local_pose.pose.position.y + self.desired_velocity.twist.linear.y #+ self.repulsion_vector.y
        desired_pose.pose.position.z = 1 + random.uniform(-0.01, 0.01) #self.local_pose.pose.position.z + self.desired_velocity.twist.linear.z #+ self.repulsion_vector.z
        desired_pose.pose.orientation = utils.rotate_quaternion_around_z(self.local_pose.pose.orientation, self.desired_velocity.twist.angular.z)
        return desired_pose

    def run(self):
        while not rospy.is_shutdown():
            desired_position = self.generate_desired_position(self.desired_velocity)
            self.position_publisher.publish(desired_position)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        converter = VelocityToPositionConverter()
        converter.run()
    except rospy.ROSInterruptException:
        pass
