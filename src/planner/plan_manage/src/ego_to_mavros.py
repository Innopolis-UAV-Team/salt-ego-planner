#!/usr/bin/env python3
# coding: utf-8

import rospy
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import Point, Vector3

class PosCmdToMavros:
    def __init__(self):
        # паблишер для MAVROS
        self.pub = rospy.Publisher(
            '/mavros/setpoint_raw/local',
            PositionTarget,
            queue_size=10
        )
        # подписка на команды траектории
        rospy.Subscriber(
            '/planning/pos_cmd',
            PositionCommand,
            self.callback,
            queue_size=1
        )

    def callback(self, msg: PositionCommand):
        sp = PositionTarget()
        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = 'map'  # или 'odom', в зависимости от вашей системы
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # type_mask — биты IGNORE_*, которые говорят FCU, какие поля НЕ учитывать.
        # Мы не ставим ни одного IGNORE_*, значит публикуем одновременно:
        # position, velocity, acceleration, yaw, yaw_rate.
        sp.type_mask = 2048 + 64 + 128 + 256

        # заполняем поля из PositionCommand
        sp.position = Point(
            x=msg.position.x,
            y=msg.position.y,
            z=msg.position.z
        )
        sp.velocity = Vector3(
            x=msg.velocity.x,
            y=msg.velocity.y,
            z=msg.velocity.z
        )
        sp.acceleration_or_force = Vector3(
            x=0,
            y=0,
            z=0
        )
        sp.yaw = msg.yaw
        sp.yaw_rate = msg.yaw_dot

        # публикуем в MAVROS
        self.pub.publish(sp)

def main():
    rospy.init_node('pos_cmd_to_mavros', anonymous=True)
    PosCmdToMavros()
    rospy.loginfo("pos_cmd_to_mavros started, waiting for /planning/pos_cmd ...")
    rospy.spin()

if __name__ == '__main__':
    main()
