#!/usr/bin/env python
"""
 * GroveUltrasonicRanger class for Grove ultrasonic ranger from Seeed Studio
 *
 * Author: Franz Pucher
 * Date (y.m.d): 2019.12.17
 * License: The MIT License (MIT)
 * 
 * Modified code from https://github.com/Seeed-Studio/Grove-RaspberryPi/blob/master/Grove%20-%20Ultrasonic%20Ranger/ultrasonic.py
"""
import RPi.GPIO as GPIO
import time

class GroveUltrasonicRanger:
    def __init__(self, i_gpio_sig = 11):
        """Initializes the ultrasonic object.
        
        Default GPIO pin is set to 11 which is used to send and receive pulse signals.
        
        Parameters:
        i_gpio_sig (int): Which pin to use for sensor signal pin
        """
        # Signal pin of the ultrasonic sensor used as output and input to measure the distance
        self.gpio_sig = i_gpio_sig
        
        # rpi board gpio or bcm gpio
        GPIO.setmode(GPIO.BOARD)

        self.elapsed = 0
        """Time it took for the emitted ultrasonic pulse to return to the sensor"""

        self.distance = 0
        """Current measured distance (unit: meter) to an obstacle"""
        
        self.distance_cm = 0
        """Current measured distance (unit: centimeter) to an obstacle"""
        

    def __del__(self):
        print "destructor: GPIO.cleanup()"
        # Reset GPIO settings
        #GPIO.cleanup() # results in AttributeException error when used in destructor
        #print "destructor: GPIO.cleanup() done"

    def getAndPrint(self):
        print "SeeedStudio Grove Ultrasonic get data and print"

        # test 100 times
        for i in range(10):
            self.measureDistance()

    def measureDistance(self):
        """Main API method to obtain a distance measurement (unit: meter)
        
        This method takes care of measuring the distance to an obstacle
        1. Prepares the sensor for a new measurement calling self.initMeasurement()
        2. Evaluates the elapsed time of the ultrasonic pulse calling self.evalMeasurement()
        3. prints distance to an obstacle (unit: meter)
        4. returns distance to an obstacle (unit: meter)
        
        Returns:
        float:Distance to an obstacle with unit meter
        
        """
        self.initMeasurement()
        self.evalMeasurement()
        print "Distance : %.3f m" % self.distance
        
        return self.distance


    def initMeasurement(self):
        """Prepare the sensor for a new measurement"""
        # setup the GPIO_SIG as output
        GPIO.setup(self.gpio_sig, GPIO.OUT)

        GPIO.output(self.gpio_sig, GPIO.LOW)
        time.sleep(0.2)
        GPIO.output(self.gpio_sig, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(self.gpio_sig, GPIO.LOW)
        start = time.time()

        # setup GPIO_SIG as input
        GPIO.setup(self.gpio_sig, GPIO.IN)

        # get duration from Ultrasonic SIG pin
        while GPIO.input(self.gpio_sig) == 0:
            start = time.time()

        while GPIO.input(self.gpio_sig) == 1:
            stop = time.time()

        # Calculate pulse length
        self.elapsed = stop - start

        return self.elapsed



    def evalMeasurement(self):
        """Evaluates a measurement storing the distance in meters and centimeters
        
        This function should be used after calling initMeasurement method.
        It uses the result of initMeasurement, the elapsed time of the ultrasonic pulse
        and converts this elapsed time in meters per second (m/s).
        This distance is the distance to an obstacle and is 
        stored in the member self.distance. After converting the result to centimeter (cm)
        it is stored in self.distance_cm.
        """
        # Distance pulse travelled in that time is time
        # multiplied by the speed of sound
        # Speed of sound in air is about 343 m/s (https://en.wikipedia.org/wiki/Speed_of_sound)
        self.distance = self.elapsed * 343.0
        #self.distance_cm = self.elapsed * 34300.0

        # That was the distance to an object and back which is why we need to halve the value
        self.distance = self.distance / 2.0
        #self.distance_cm = self.distance_cm / 2.0
        self.distance_cm = self.distance * 100.0


if __name__ == '__main__':
    try:  
        # construct ultrasonic sensor
        sensor = GroveUltrasonicRanger()

        # loop and print the distances to obstacles
        sensor.getAndPrint()
  
    except KeyboardInterrupt:  
        # here you put any code you want to run before the program   
        # exits when you press CTRL+C  
        print "KeyboardInterrupt"  
  
    except:  
        # this catches ALL other exceptions including errors.  
        # You won't get any error messages for debugging  
        # so only use it once your code is working  
        print "Other error or exception occurred!"  
  
    finally:
        print "GPIO.cleanup()"
        GPIO.cleanup() # this ensures a clean exit
        print "GPIO.cleanup() done"
