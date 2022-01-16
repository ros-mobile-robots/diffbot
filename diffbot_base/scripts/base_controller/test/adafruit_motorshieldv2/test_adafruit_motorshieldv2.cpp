/*
 * Test for Adafruit motor shield v2 library
 * This can be used to test the Adafruit Stepper + DC Motor FeatherWing
 * https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing
 * 
 * Test base on the following example
 * https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library/blob/master/examples/DCMotorTest/DCMotorTest.ino
 * 
 * Author: Franz Pucher
 * 
 */

// Include unity library
// For details see https://docs.platformio.org/en/latest/plus/unit-testing.html#api
#include <unity.h>

// The special bit of code in this example is the use of Arduino's Wire library. 
// Wire is a I2C library that simplifies reading and writing to the I2C bus.
#include <Wire.h>
#include <Adafruit_MotorShield.h>

#include "diffbot_base_config.h"


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(MOTOR_DRIVER_ADDR);
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *rightMotor = AFMS.getMotor(MOTOR_RIGHT);
// You can also make another motor on port M2
Adafruit_DCMotor *leftMotor = AFMS.getMotor(MOTOR_LEFT);


void motorSweep(Adafruit_DCMotor *motor)
{
  uint8_t i;
  motor->run(FORWARD);
  for (i=0; i<255; i++) {
    motor->setSpeed(i);  
    delay(10);
  }
  for (i=255; i!=0; i--) {
    motor->setSpeed(i);  
    delay(10);
  }
  
  Serial.print("tock");

  motor->run(BACKWARD);
  for (i=0; i<255; i++) {
    motor->setSpeed(i);  
    delay(10);
  }
  for (i=255; i!=0; i--) {
    motor->setSpeed(i);  
    delay(10);
  }

  Serial.print("tech");
  motor->run(RELEASE);
  delay(1000);
}

void setup() {
  // NOTE!!! Wait for >2 secs
  // if board doesn't support software reset via Serial.DTR/RTS
  delay(2000);
  UNITY_BEGIN();

  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  

  // Set the speed to start, from 0 (off) to 255 (max speed)
  Serial.println("setSpeed(150)");
  leftMotor->setSpeed(150);
  leftMotor->run(FORWARD);
  // turn on motor
  leftMotor->run(RELEASE);

  // Set the speed to start, from 0 (off) to 255 (max speed)
  rightMotor->setSpeed(150);
  rightMotor->run(FORWARD);
  // turn on motor
  rightMotor->run(RELEASE);
}

uint8_t cycle = 0;
uint8_t max_loop_cycles = 1;

void loop() {

  if (cycle < max_loop_cycles)
  {
    Serial.print("tick");

    motorSweep(leftMotor);
    motorSweep(rightMotor);
  }
  else
  {
    leftMotor->setSpeed(0); 
    leftMotor->run(RELEASE);
    rightMotor->setSpeed(0); 
    rightMotor->run(RELEASE);
    delay(1000);
    UNITY_END();
  }

  cycle++;
}