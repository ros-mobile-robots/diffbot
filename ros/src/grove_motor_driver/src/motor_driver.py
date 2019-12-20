#!/usr/bin/env python

from grove_i2c_motor_driver import motor_driver

import rospy
from std_msgs.msg import Float32

from geometry_msgs.msg import Twist


def twist_callback(msg):
    if (msg.linear.x > 0):
        motor.MotorDirectionSet(0b1010)
        motor.MotorSpeedSetAB(100,100)
    elif (msg.linear.x < 0):
        motor.MotorDirectionSet(0b1010)
        motor.MotorSpeedSetAB(100,100)
    else:
        motor.MotorSpeedSetAB(0,0)
    

if __name__ == '__main__':
    motor = motor_driver() # <1>

    # Initialize the node
    rospy.init_node('motor_driver')

    # Topic for the volume
    t = rospy.Subscriber('twist', Twist, twist_callback)

    # Start everything
    rospy.spin()
