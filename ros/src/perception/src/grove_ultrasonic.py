#!/usr/bin/env python

import RPi.GPIO as GPIO
import time

class Ultrasonic:
    def __init__(self, i_gpio_sig = 11):
        # Signal pin of the ultrasonic sensor used as output and input to measure the distance
        self.gpio_sig = i_gpio_sig
        
        # rpi board gpio or bcm gpio
        GPIO.setmode(GPIO.BOARD)

        # Time it took for the emitted ultrasonic pulse to return to the sensor
        self.elapsed = 0

        # Current distance to an obstacle
        self.distance = 0


    def getAndPrint(self):
        print "SeeedStudio Grove Ultrasonic get data and print"

        # test 100 times
        for i in range(100):
            self.measureDistance()

        # Reset GPIO settings
        GPIO.cleanup()

    def measureDistance(self):
        self.initMeasurement()
        self.evalMeasurement()
        print "Distance : %.3f m" % self.distance
        
        return self.distance

    def initMeasurement(self):
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
        
        # Distance pulse travelled in that time is time
        # multiplied by the speed of sound
        # Speed of sound in air is about 343 m/s (https://en.wikipedia.org/wiki/Speed_of_sound)
        self.distance = self.elapsed * 343.0

        # That was the distance to an object and back which is why we need to halve the value
        self.distance = self.distance / 2.0

        return self.distance
        

    def evalMeasurementCM(self):
        print "Ultrasonic Measurement"

        # Distance pulse travelled in that time is time
        # multiplied by the speed of sound (cm/s)
        # Speed of sound in air is about 343 m/s (https://en.wikipedia.org/wiki/Speed_of_sound)
        self.distance = self.elapsed * 34300.0
        

        # That was the distance to an object and back which is why we need to halve the value
        self.distance = self.distance / 2.0

        return self.distance


if __name__ == '__main__':
    
    sensor = Ultrasonic()

    # loop method
    sensor.getAndPrint()
