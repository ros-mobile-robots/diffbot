"""
Optical Sensor Two Motor Demonstration
DualMotorSpeedDemo.ino
Demonstrates use of Hardware Interrupts
to measure speed from two motors
	
DroneBot Workshop 2017
http://dronebotworkshop.com
"""

# Use time module to sleep for specified time
import time

# RPi.GPIO module
# https://sourceforge.net/p/raspberry-gpio-python/wiki/BasicUsage/
import RPi.GPIO as GPIO
# Use Board numbering scheme (physical pin numbers) instead of 
# BCM (channel numbers on the Broadcom SOC) to avoid re-wiring due to board revision changes.
GPIO.setmode(GPIO.BOARD) 


# Constants for Interrupt Pins
# Speed sensor 1 interrupt pin uses pin BCM 22, Physical 15
SPEED_SENSOR_LEFT  = 15  
# Speed sensor 2 interrupt pin uses pin BCM 23, Physical 16
SPEED_SENSOR_RIGHT = 16

# GPIO 15 & 16 set up as inputs. Both are pulled up.
# The pins will go to GND when the encoder wheels spin. 
# They are located in the gap of the H206 slot-type opto interrupters,
# between LED and phototransistor sensor. 
# Rotating motors cause the the encoder wheels to spin and produce 
# pulses of light when light travels from the LED to the transistor.
# This causes the transistor to switch on and off, leading to high/low transitions.
# between the optocoupler  spins pressed and 24 will go to 3V3 (3.3V)
# this enables us to demonstrate both rising and falling edge detection
GPIO.setup(SPEED_SENSOR_LEFT , GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(SPEED_SENSOR_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Integers for pulse counters
counter_left = 0
counter_right = 0

# Float for number of slots in encoder disk
diskslots = 20.0  # Change to match value of encoder disk

# Interrupt service routines (ISR)

# Pulse count ISR for left speed sensor (motor)
def ISR_count_left(channel):
    global counter_left
    # Increment speed sensor left counter value
    counter_left += 1  
    #print(counter_left)


# Pulse count ISR for right speed sensor (motor)
def ISR_count_right(channel):
    global counter_right
    # Increment speed sensor right counter value
    counter_right += 1  


# Timer ISR
def ISR_timer():
    global counter_left
    global counter_right

    # Calculate RPM for left motor
    rotation_left = (counter_left / diskslots) * 60.00
    print("Motor Speed 1:", rotation_left, "RPM")  
    counter_left = 0  # Reset counter to zero
    
    # Calculate RPM for right motor
    rotation_right = (counter_right / diskslots) * 60.00
    print("Motor Speed 2: ", rotation_right, "RPM")
    counter_right = 0;  # Reset counter to zero
    

# When a falling edge is detected on the interrupt pins, regardless of whatever   
# else is happening in the program, the function ISR_count_left and ISR_count_right will be run
# Increase counter left when speed sensor pin goes high
GPIO.add_event_detect(SPEED_SENSOR_LEFT, GPIO.RISING, callback=ISR_count_left)#, bouncetime=1)
# Increase counter right when speed sensor pin goes high
GPIO.add_event_detect(SPEED_SENSOR_RIGHT, GPIO.RISING, callback=ISR_count_right)#, bouncetime=1)


def main():
    try:
        while True:
            ISR_timer()
            # Set sleep timer for 1sec
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exit") 


if __name__=="__main__":
    main()
