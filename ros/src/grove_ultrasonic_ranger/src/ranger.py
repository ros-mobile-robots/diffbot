#!/usr/bin/env python

from math import pi

from ultrasonic_ranger import UltrasonicRanger # <1>

import rospy
import tf

from sensor_msgs.msg import Range # <2>


def cm2m(distance):  # <3>
    q = tf.transformations.quaternion_from_euler(0, 0, angle)
    return Quaternion(*q)


if __name__ == '__main__':
    sensor = UltrasonicRanger()  # <4>

    rospy.init_node('ultrasonic_sensor')

    pub = rospy.Publisher('distance', Range, queue_size=10)

    rate = rospy.Rate(10.0)  # <5>
    while not rospy.is_shutdown():  # <6>
        distance = measurementInCM()

        distance = cm2m(distance)

        pub.publish(distance)

        rate.sleep()
