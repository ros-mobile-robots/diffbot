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

# Constants for Interrupt Pins
# Speed sensor 1 interrupt pin uses pin BCM 22, Physical 15
SPEED_SENSOR_LEFT  = 15  
# Speed sensor 2 interrupt pin uses pin BCM 23, Physical 16
SPEED_SENSOR_RIGHT = 16

# Float for number of slots in encoder disk 
# Use float because we divide by this number
DISKSLOT = 20.0

# Update rate (in seconds) of the speed sensors
UPDATE_RATE = 1.0

class LM393SpeedSensor:
	def __init__(self, gpio_mode=GPIO.BOARD, 
		     gpio_sensor_left=SPEED_SENSOR_LEFT, 
		     gpio_sensor_right=SPEED_SENSOR_RIGHT,
		     diskslots=DISKSLOT,
             update_rate=UPDATE_RATE):
		
        self.gpio_mode = gpio_mode

        # Use Board numbering scheme (physical pin numbers) instead of 
        # BCM (channel numbers on the Broadcom SOC) to avoid re-wiring due to board revision changes.
        GPIO.setmode(self.gpio_mode)
		
        # Speed sensor 1 interrupt pin. Default pin BCM 22, Physical 15
        self.gpio_sensor_left = gpio_sensor_left
        # Speed sensor 2 interrupt pin. Default pin BCM 23, Physical 16
        self.gpio_sensor_right = gpio_sensor_right
		
		
		# GPIO 15 & 16 set up as inputs. Both are pulled up.
		# The pins will go to GND when the encoder wheels spin. 
		# They are located in the gap of the H206 slot-type opto interrupters,
		# between LED and phototransistor sensor. 
		# Rotating motors cause the the encoder wheels to spin and produce 
		# pulses of light when light travels from the LED to the transistor.
		# This causes the transistor to switch on and off, leading to high/low transitions.
		GPIO.setup(self.gpio_sensor_left , GPIO.IN, pull_up_down=GPIO.PUD_UP)
		GPIO.setup(self.gpio_sensor_right, GPIO.IN, pull_up_down=GPIO.PUD_UP)

		
		# Integers for pulse counters
		self.counter_left = 0
		self.counter_right = 0

		
		# Float for number of slots in encoder disk
        self.diskslots = 20.0  # Change to match value of encoder disk
        
        # Update rate (in seconds) of the speed sensors
        self.update_rate = update_rate
		
        # When a falling edge is detected on the interrupt pins, regardless of whatever   
        # else is happening in the program, the function ISR_count_left and ISR_count_right will be run
        # Increase counter left when speed sensor pin goes high
        GPIO.add_event_detect(self.gpio_sensor_left, GPIO.RISING, callback=ISR_count_left)#, bouncetime=1)
        # Increase counter right when speed sensor pin goes high
        GPIO.add_event_detect(self.gpio_sensor_right, GPIO.RISING, callback=ISR_count_right)#, bouncetime=1)

	
    
	# Interrupt service routines (ISR)

	# Pulse count ISR for left speed sensor (motor)
	def ISR_count_left(self, channel):
	    # Increment speed sensor left counter value
	    self.counter_left += 1  
	    #print(counter_left)


	# Pulse count ISR for right speed sensor (motor)
	def ISR_count_right(self, channel):
	    # Increment speed sensor right counter value
	    self.counter_right += 1  


	# Timer ISR
	def ISR_timer(self):

	    # Calculate RPM for left motor
	    rotation_left = (self.counter_left / self.diskslots) * self.update_rate * 60.00
	    print("Motor Speed 1:", rotation_left, "RPM")  
	    counter_left = 0  # Reset counter to zero

	    # Calculate RPM for right motor
	    rotation_right = (self.counter_right / self.diskslots) * self.update_rate * 60.00
	    print("Motor Speed 2: ", rotation_right, "RPM")
	    counter_right = 0;  # Reset counter to zero
        
        return rotation_left, rotation_right

		
	def run(self):
        try:
            while True:
                ISR_timer()
                # Set sleep timer for self.update_rate seconds. Default 1 second
                time.sleep(self.update_rate)
        except KeyboardInterrupt:
            print("Exit") 
		


def main():
    lm393SpeedSensor = LM393SpeedSensor()
    lm393SpeedSensor.run()


if __name__=="__main__":
    main()
