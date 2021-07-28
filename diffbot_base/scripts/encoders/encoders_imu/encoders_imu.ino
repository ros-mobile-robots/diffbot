/* Encoder Library - TwoKnobs Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */


#include <ros.h>
//#include <std_msgs/Int32.h>
#include <diffbot_msgs/Encoders.h>
#include <diffbot_msgs/IMU.h>
#include <std_msgs/Empty.h>

#include <Encoder.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

double orientation[4];
double angular_velocity[3];
double linear_acceleration[3];

// Encoder setup
// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder encoderLeft(5, 6);  // Default pins 5, 6
Encoder encoderRight(7, 8); // Default pins 7, 8
//   avoid using pins with LEDs attached

ros::NodeHandle nh;

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

// This uses the custom IMU imu_msg message that defines an array of 10 integers
// The first 4 integers for Orientation (4th is Z angular velocity, becuse <sensor_msgs/Imu.h> structure)
// Next 3 integers for Acceleration
// Next 3 integers for Angular Velocity
diffbot_msgs::IMU imu_data;
ros::Publisher pub_imu("imu", &imu_data);

void setup()
{

  // Encoder
  nh.initNode();
  nh.advertise(pub_encoders);
  nh.subscribe(sub_reset);

  while (!nh.connected())
  {
    nh.spinOnce();
  }
  nh.loginfo("Initialize DiffBot Wheel Encoders");
  std_msgs::Empty reset;
  resetCallback(reset);
  delay(1);

  // IMU
  nh.advertise(pub_imu);
  nh.loginfo("Initialize Diffbot Axis Data");
  delay(1);
  bno.begin();
  //if (!bno.begin())
  //{
  //  /* There was a problem detecting the BNO055 ... check your connections */
  //  Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  //  while (1)
  //    ;
  //}
}

long positionLeft  = -999;
long positionRight = -999;

void loop() {
  long newLeft, newRight;
  newLeft = encoderLeft.read();
  newRight = encoderRight.read();

  encoder_ticks.ticks[0] = newLeft;
  encoder_ticks.ticks[1] = newRight;
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
 
  // Note that https://www.arduino.cc/reference/en/language/functions/time/delay/:
  // Certain things do go on while the delay() function is controlling the Atmega chip,
  // however, because the delay function does not disable interrupts.
  // Serial communication that appears at the RX pin is recorded, PWM (analogWrite) values and pin states are maintained,
  // and interrupts will work as they should.

  // IMU CODE SECTION

  // Get a new IMU sensor event //
  sensors_event_t orien, accel, gyro;
  bno.getEvent(&orien, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&accel), Adafruit_BNO055::VECTOR_LINEARACCEL;
  bno.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  // Bulding IMU message into array that will get publish to ROS
  imu_data.imu_msg[0] = orien.orientation.x;
  imu_data.imu_msg[1] = orien.orientation.y;
  imu_data.imu_msg[2] = orien.orientation.z;
  imu_data.imu_msg[3] = accel.acceleration.x;
  imu_data.imu_msg[4] = accel.acceleration.y;
  imu_data.imu_msg[5] = accel.acceleration.z;
  imu_data.imu_msg[6] = gyro.gyro.x;
  imu_data.imu_msg[7] = gyro.gyro.y;
  imu_data.imu_msg[8] = gyro.gyro.z;
  imu_data.imu_msg[9] = bno.getTemp();

  // Publishing Nodes
  pub_imu.publish(&imu_data);
  pub_encoders.publish(&encoder_ticks);
  nh.spinOnce();
  delay(5);

  if (newLeft != positionLeft || newRight != positionRight)
  {
    //String str = "Left = " + String(newLeft) + ", Right = " + String(newRight);
    //nh.loginfo(str);
    positionLeft = newLeft;
    positionRight = newRight;
  }

}