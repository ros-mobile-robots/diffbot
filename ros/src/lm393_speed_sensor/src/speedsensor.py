"""
Optical Sensor Two Motor Demonstration
DualMotorSpeedDemo.ino
Demonstrates use of Hardware Interrupts
to measure speed from two motors
	
DroneBot Workshop 2017
http://dronebotworkshop.com
"""

## Include the TimerOne Library from Paul Stoffregen
#include "TimerOne.h"
import time

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)


# Constants for Interrupt Pins
# Change values if not using Arduino Uno

#const byte MOTOR1 = 2;  // Motor 1 Interrupt Pin - INT 0
#const byte MOTOR2 = 3;  // Motor 2 Interrupt Pin - INT 1

# GPIO 23 & 24 set up as inputs. One pulled up, the other down.
# 23 will go to GND when button pressed and 24 will go to 3V3 (3.3V)
# this enables us to demonstrate both rising and falling edge detection
GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Integers for pulse counters
counter1 = 0
counter2 = 0

# Float for number of slots in encoder disk
diskslots = 20.0  # Change to match value of encoder disk

# Interrupt Service Routines

# Motor 1 pulse count ISR
def ISR_count1(channel):
    global counter1
    counter1 += 1  # increment Motor 1 counter value
    #print(counter1)


# Motor 2 pulse count ISR
def ISR_count2(channel):
    global counter2
    counter2 += 1  # increment Motor 2 counter value


# Timer ISR
def ISR_timer():
    #Timer1.detachInterrupt()  # Stop the timer
    global counter1
    global counter2

    rotation1 = (counter1 / diskslots) * 60.00  # calculate RPM for Motor 1
    print("Motor Speed 1:", rotation1, "RPM -")  
    counter1 = 0  # reset counter to zero
    
    rotation2 = (counter2 / diskslots) * 60.00;  # calculate RPM for Motor 2
    #print("Motor Speed 2: ", rotation2, "RPM")
    counter2 = 0;  # reset counter to zero
    
    #Timer1.attachInterrupt(ISR_timerone);  # Enable the timer
    

# when a falling edge is detected on port 17, regardless of whatever   
# else is happening in the program, the function my_callback will be run  
GPIO.add_event_detect(22, GPIO.RISING, callback=ISR_count1, bouncetime=1)


def main():
    try:
        while True:
            ISR_timer()
            # Set timer for 1sec
            time.sleep(1)
            # Increase counter 1 when speed sensor pin goes High
            #attachInterrupt(digitalPinToInterrupt (MOTOR1), ISR_count1, RISING);  
            # Increase counter 2 when speed sensor pin goes High
            #attachInterrupt(digitalPinToInterrupt (MOTOR2), ISR_count2, RISING);  
            # Enable the timer
            #Timer1.attachInterrupt( ISR_timerone ); // Enable the timer
            #print("Iteration")
    except KeyboardInterrupt:
        print("Exit") 


if __name__=="__main__":
    main()
