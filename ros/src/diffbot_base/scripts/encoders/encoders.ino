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
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  // reset both back to zero.
  encoderLeft.write(0);
  encoderRight.write(0);
  if (Serial.available()) {
    Serial.println("Reset both wheel encoders to zero");
  }
}

ros::Subscriber<std_msgs::Empty> sub_reset("/reset", resetCallback);

// ROS Publisher setup to publish left and right encoder ticks
std_msgs::Int32 int_ticksLeft;
std_msgs::Int32 int_ticksRight;
ros::Publisher pub_encoderLeft("/diffbot/ticks_left", &int_ticksLeft);
ros::Publisher pub_encoderRight("/diffbot/ticks_right", &int_ticksRight);

void setup() {
  Serial.begin(115200);
  Serial.println("DiffBot Wheel Encoders:");

  nh.initNode();
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
  //delay(500);
  
  if (newLeft != positionLeft || newRight != positionRight) {
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.print(", Right = ");
    Serial.print(newRight);
    Serial.println();
    positionLeft = newLeft;
    positionRight = newRight;
  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both wheel encoders to zero");
    encoderLeft.write(0);
    encoderRight.write(0);
  }
}
