#!/usr/bin/env python

from math import pi

from grove_ultrasonic_ranger import GroveUltrasonicRanger # <1>

import rospy
import tf

from sensor_msgs.msg import Range # <2>


if __name__ == '__main__':
    sensor = GroveUltrasonicRanger()  # <4>

    rospy.init_node('ranger')

    pub = rospy.Publisher('distance', Range, queue_size=10)

    rate = rospy.Rate(10.0)  # <5>
    while not rospy.is_shutdown():  # <6>
        distance = sensor.measureDistance()

        pub.publish(distance)

        rate.sleep()
