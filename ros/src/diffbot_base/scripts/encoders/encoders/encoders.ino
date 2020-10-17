/* Encoder Library - TwoKnobs Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>

#include <Encoder.h>

// Encoder setup
// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder encoderLeft(5, 6);  // Default pins 5, 6
Encoder encoderRight(7, 8); // Default pins 7, 8
//   avoid using pins with LEDs attached

ros::NodeHandle  nh;

// ROS Subscriber setup to reset both encoders to zero
void resetCallback( const std_msgs::Empty& reset){
  //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  // reset both back to zero.
  encoderLeft.write(0);
  encoderRight.write(0);
  nh.loginfo("Reset both wheel encoders to zero");
}

ros::Subscriber<std_msgs::Empty> sub_reset("/reset", resetCallback);

// ROS Publisher setup to publish left and right encoder ticks
std_msgs::Int32 int_ticksLeft;
std_msgs::Int32 int_ticksRight;
ros::Publisher pub_encoderLeft("/diffbot/ticks_left", &int_ticksLeft);
ros::Publisher pub_encoderRight("/diffbot/ticks_right", &int_ticksRight);

void setup() 
{
  nh.initNode();
  nh.loginfo("DiffBot Wheel Encoders:");
  nh.advertise(pub_encoderLeft);
  nh.advertise(pub_encoderRight);
  nh.subscribe(sub_reset);
}

long positionLeft  = -999;
long positionRight = -999;

void loop() {
  long newLeft, newRight;
  newLeft = encoderLeft.read();
  newRight = encoderRight.read();

  int_ticksLeft.data = newLeft;
  int_ticksRight.data = newRight;
  pub_encoderLeft.publish(&int_ticksLeft);
  pub_encoderRight.publish(&int_ticksRight);
  nh.spinOnce();
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
  delay(5);
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
