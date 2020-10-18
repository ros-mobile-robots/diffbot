#!/usr/bin/env python

from grove_i2c_motor_driver import GroveMotorDriver

import rospy
from std_msgs.msg import Int32


class MotorDriver:

    def __init__(self, address=0x0f):
        self.motor_driver = GroveMotorDriver(address=0x0f)
        #"0b1001" defines the output polarity of both motors
        # The first two least significant bits "01" are for the right motor 
        # and the following bits "10" specify the polarity of the left motor,
        # "10" means the M+ is "positive
        self.motor = {
            'left': {
                'dir': 0b10,
                'val': 0,
            },
            'right': {
                'dir': 0b01,
                'val': 0,
            },
        }

    def left_motor_callback(self, msg):
        #"0b1010" defines the output polarity, "10" means the M+ is "positive
        if (msg.data > 0):
            self.motor['left']['dir'] = 0b10
            self.motor['left']['val'] = min(msg.data, 100)
            rospy.loginfo(f"Left forward: {self.motor['left']['val']}")
        elif (msg.data < 0):
            self.motor['left']['dir'] = 0b01
            self.motor['left']['val'] = min(-msg.data, 100)
            rospy.loginfo(f"Left backward: {self.motor['left']['val']}")
        else:
            self.motor['left']['val'] = 0
            rospy.loginfo(f"Left stop: {self.motor['left']['val']}")

        self.update_motors()


    def right_motor_callback(self, msg):
        #"0b1010" defines the output polarity, "10" means the M+ is "positive
        if (msg.data > 0):
            self.motor['right']['dir'] = 0b01
            self.motor['right']['val'] = min(msg.data, 100)
            rospy.loginfo(f"Right forward: {self.motor['right']['val']}")
        elif (msg.data < 0):
            self.motor['right']['dir'] = 0b10
            self.motor['right']['val'] = min(-msg.data, 100)
            rospy.loginfo(f"Right backward: {self.motor['right']['val']}")
        else:
            self.motor['right']['val'] = 0
            rospy.loginfo(f"Right stop: {self.motor['right']['val']}")

        self.update_motors()


    def update_motors(self):
        direction = self.motor['left']['dir']<<2 | self.motor['right']['dir']
        self.motor_driver.MotorDirectionSet(direction)
        self.motor_driver.MotorSpeedSetAB(self.motor['right']['val'], self.motor['left']['val'])

    def stop_motors(self):
        self.motor_driver.MotorSpeedSetAB(0, 0)


if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('motor_driver')

    motor_driver = MotorDriver(address=0x0f)

    # Topics for the motors
    sub_left = rospy.Subscriber('motor_left', Int32, motor_driver.left_motor_callback)
    sub_right = rospy.Subscriber('motor_right', Int32, motor_driver.right_motor_callback)

    rospy.loginfo("Ready to receive motor commands")

    # Start everything
    # This will block until the node is killed
    # Callbacks run in their own thread and will still work
    rospy.spin()

    rospy.loginfo("Stop motors")
    motor_driver.stop_motors()
