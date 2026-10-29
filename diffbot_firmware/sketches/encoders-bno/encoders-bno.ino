/* Encoder Library - TwoKnobs Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */
// Adafruit BNO055 code added by Russ76
// Red to 3.3V, Black GRND, Yellow SCL 19, Blue SDA 18
/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include "ros/time.h"
//#include <std_msgs/Int32.h>
#include <diffbot_msgs/Encoders.h>
#include <std_msgs/Empty.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "sensor_msgs/Imu.h"
#include <Encoder.h>

// Encoder setup
// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder encoderLeft(5, 6);  // Default pins 5, 6
Encoder encoderRight(7, 8); // Default pins 7, 8
//   avoid using pins with LEDs attached
char imu_link[] = "imu";

ros::NodeHandle  nh;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
sensor_msgs::Imu Imu_msg; 
ros::Publisher imu_pub("imu/data_raw", &Imu_msg);

// ROS Subscriber setup to reset both encoders to zero
void resetCallback( const std_msgs::Empty& reset)
{
  //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  // reset both back to zero.
  encoderLeft.write(0);
  encoderRight.write(0);
  nh.loginfo("Reset both wheel encoders to zero");
}

ros::Subscriber<std_msgs::Empty> sub_reset("reset", resetCallback);

// ROS Publisher setup to publish left and right encoder ticks
// This uses the custom encoder ticks message that defines an array of two integers
diffbot_msgs::Encoders encoder_ticks;
ros::Publisher pub_encoders("encoder_ticks", &encoder_ticks);

void setup() 
{
  nh.initNode();
  nh.advertise(pub_encoders);
  nh.subscribe(sub_reset);
  nh.advertise(imu_pub);
  while (!nh.connected())
  {
    nh.spinOnce();
  }
  nh.loginfo("Initialize DiffBot Wheel Encoders");
  std_msgs::Empty reset;
  resetCallback(reset);
  delay(1);
  
    /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(100);
  bno.setExtCrystalUse(true);
}

long positionLeft  = -999;
long positionRight = -999;

void loop() {  
  long newLeft, newRight;
  newLeft = encoderLeft.read();
  newRight = encoderRight.read();

  encoder_ticks.ticks[0] = newLeft;
  encoder_ticks.ticks[1] = newRight;
  pub_encoders.publish(&encoder_ticks);
  nh.spinOnce();
  
       
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Quaternion quat = bno.getQuat();
  
  //nh.loginfo("In main loop now.");
	
  // http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Imu.html
  sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = imu_link;
    imu_msg.header.stamp = nh.now();
    imu_msg.orientation.x = quat.w();
    imu_msg.orientation.y = quat.x();
    imu_msg.orientation.z = quat.y();
    imu_msg.orientation.w = quat.z();
    imu_msg.linear_acceleration.x = (euler.x());
    imu_msg.linear_acceleration.y = (euler.y());
    imu_msg.linear_acceleration.z = (euler.z());
    imu_msg.angular_velocity.x = (gyro.x());
    imu_msg.angular_velocity.y = (gyro.y());
    imu_msg.angular_velocity.z = (gyro.z());
  // https://github.com/Russ76/ros_mpu6050_node-1/blob/master/src/mpu6050_node.cpp
  imu_pub.publish(&imu_msg);
		
  
  // Use at least a delay of 3 ms on the work pc and 5 ms on the Raspberry Pi
  // Too low delay causes errors in rosserial similar to the following:
  // [INFO]: wrong checksum for topic id and msg
  // [INFO]: Wrong checksum for msg length, length 4, dropping message.
  // [ERROR]: Mismatched protocol version in packet (b'\x00'): lost sync or rosserial_python is from different ros release than the rosserial client
  // [INFO]: Protocol version of client is unrecognized, expected Rev 1 (rosserial 0.5+)
  // [INFO]: wrong checksum for topic id and msg
  // [WARN]: Last read step: message length
  // [WARN]: Run loop error: Serial Port read failure: Returned short (expected 3 bytes, received 2 instead).
  // [INFO]: Requesting topics...
  // [ERROR]: Lost sync with device, restarting...
  // [INFO]: Requesting topics...
  delay(10);
  // Note that https://www.arduino.cc/reference/en/language/functions/time/delay/:
  // Certain things do go on while the delay() function is controlling the Atmega chip, 
  // however, because the delay function does not disable interrupts. 
  // Serial communication that appears at the RX pin is recorded, PWM (analogWrite) values and pin states are maintained, 
  // and interrupts will work as they should.
  
  if (newLeft != positionLeft || newRight != positionRight) {
    //String str = "Left = " + String(newLeft) + ", Right = " + String(newRight);
    //nh.loginfo(str);
    positionLeft = newLeft;
    positionRight = newRight;
  }
}
