#!/usr/bin/env python

from grove_i2c_motor_driver import MotorDriver

import rospy
from std_msgs.msg import Float32

from geometry_msgs.msg import Twist


def twist_callback(msg):
    if (msg.linear.x > 0.0):
        print("Forward")
        motor.MotorDirectionSet(0b1010)
        motor.MotorSpeedSetAB(100,100)
    elif (msg.linear.x < 0.0):
        print("Back")
        motor.MotorDirectionSet(0b0101)
        motor.MotorSpeedSetAB(100,100)
    elif (-0.1 < msg.linear.x and msg.linear.x < 0.1):
        print("Stop Delta")
        motor.MotorSpeedSetAB(0,0)
    else:
        print("Stop")
        motor.MotorSpeedSetAB(0,0)
    

if __name__ == '__main__':
    motor = MotorDriver() # <1>

    # Initialize the node
    rospy.init_node('motor_driver')

    # Topic for the volume
    t = rospy.Subscriber('twist', Twist, twist_callback)

    # Start everything
    rospy.spin()
