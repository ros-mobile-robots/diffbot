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
    
    # create default message of type sensor_msg/Range with default values
    # http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Range.html
    msg = Range()
    msg.header.frame_id = "ranger_distance"
    # http://wiki.seeedstudio.com/Grove-Ultrasonic_Ranger/#specification
    msg.field_of_view = 15
    msg.min_range = 0.02 # m
    msg.max_range = 3.5  # m

    rate = rospy.Rate(10.0)  # <5>
    while not rospy.is_shutdown():  # <6>
        range = sensor.measureDistance()
        
        # update time stamp of default message
        msg.header.stamp = rospy.Time.now()
        
        # update measured distance to obstacle
        msg.range = range
        
        # publish the updated message
        pub.publish(msg)

        rate.sleep()
