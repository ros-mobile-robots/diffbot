#include <Arduino.h>

#include <ros.h>
//#include <std_msgs/Int32.h>
#include <diffbot_msgs/Encoders.h>
#include <diffbot_msgs/WheelCmd.h>
#include <std_msgs/Empty.h>

#include "diffbot_base_config.h"
#include "encoder.h"
#include "adafruit_feather_wing/adafruit_feather_wing.h"


#define IMU_PUBLISH_RATE 1 //hz
#define COMMAND_RATE 20 //hz
#define DEBUG_RATE 5
#define DEBUG 1

#define ENCODER_RESOLUTION 542


// Encoder setup
// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
diffbot::Encoder encoderLeft(ENCODER_LEFT_H1, ENCODER_LEFT_H2, ENCODER_RESOLUTION);  // Default pins 5, 6
diffbot::Encoder encoderRight(ENCODER_RIGHT_H1, ENCODER_RIGHT_H2, ENCODER_RESOLUTION); // Default pins 7, 8
//   avoid using pins with LEDs attached

AdafruitMotorController motor_controller_right = AdafruitMotorController(3);
MotorControllerIntf<Adafruit_MotorShield>* p_motor_controller_right = &motor_controller_right;
AdafruitMotorController motor_controller_left = AdafruitMotorController(4);
MotorControllerIntf<Adafruit_MotorShield>* p_motor_controller_left = &motor_controller_left;

ros::NodeHandle  nh;

void eStop();
void read();
void write();
void printDebug();

void commandCallback(const diffbot_msgs::WheelCmd& cmd_msg);

ros::Subscriber<diffbot_msgs::WheelCmd> sub_wheel_cmd_velocities("wheel_cmd_velocities", commandCallback);

float g_wheel_cmd_velocity_left = 0;
float g_wheel_cmd_velocity_right = 0;

unsigned long g_prev_command_time = 0;

unsigned long g_encoder_resolution = 0;
float g_wheel_radius = 0;
float g_max_linear_velocity = 0;
float g_max_angular_velocity = 0;

void commandCallback(const diffbot_msgs::WheelCmd& cmd_msg)
{
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored
    g_wheel_cmd_velocity_left = cmd_msg.velocities[0];
    g_wheel_cmd_velocity_right = cmd_msg.velocities[1];

    g_prev_command_time = millis();
}


// ROS Subscriber setup to reset both encoders to zero
void resetCallback(const std_msgs::Empty& reset)
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
diffbot_msgs::Encoders encoders;
ros::Publisher pub_encoders("encoder_ticks", &encoders);

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.advertise(pub_encoders);
  nh.subscribe(sub_wheel_cmd_velocities);
  nh.subscribe(sub_reset);

  while (!nh.connected())
  {
    nh.spinOnce();
  }


  nh.loginfo("Initialize DiffBot Motor Controllers");
  motor_controller_left.begin();
  motor_controller_right.begin();

  nh.loginfo("Get Parameters from Parameter Server");
  nh.getParam("/diffbot/encoder_resolution", &g_encoder_resolution);
  String log_msg = String("/diffbot/encoder_resolution: ") + String(g_encoder_resolution);
  nh.loginfo(log_msg.c_str());
  nh.getParam("/diffbot/mobile_base_controller/wheel_radius", &g_wheel_radius);
  log_msg = String("/diffbot/mobile_base_controller/wheel_radius: ") + String(g_wheel_radius);
  nh.loginfo(log_msg.c_str());
  nh.getParam("/diffbot/mobile_base_controller/linear/x/max_velocity", &g_max_linear_velocity);
  log_msg = String("/diffbot/mobile_base_controller/linear/x/max_velocity: ") + String(g_max_linear_velocity);
  nh.loginfo(log_msg.c_str());

  nh.loginfo("Initialize DiffBot Wheel Encoders");
  std_msgs::Empty reset;
  resetCallback(reset);
  delay(1);

  g_max_angular_velocity = g_max_linear_velocity / g_wheel_radius;

  delay(5000);
  
}

long positionLeft  = -999;
long positionRight = -999;

void loop() 
{
  static unsigned long prev_control_time = 0;
  static unsigned long prev_imu_time = 0;
  static unsigned long prev_debug_time = 0;
  static bool imu_is_initialized;

  //this block drives the robot based on defined rate
  if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
  {
    read();
    write();
    prev_control_time = millis();
  }

  //this block stops the motor when no command is received
  if ((millis() - g_prev_command_time) >= 400)
  {
    eStop();
  }

  //this block publishes the IMU data based on defined rate
  if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
  {
    //sanity check if the IMU is connected
    if (!imu_is_initialized)
    {
      //imu_is_initialized = initIMU();

      if(imu_is_initialized)
        nh.loginfo("IMU Initialized");
      else
        nh.logfatal("IMU failed to initialize. Check your IMU connection.");
    }
    else
    {
        //publishIMU();
    }
    prev_imu_time = millis();
  }

  //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
  if(DEBUG)
  {
    if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
    {
      printDebug();
      prev_debug_time = millis();
    }
  }
  //call all the callbacks waiting to be called
  nh.spinOnce();
}


void read()
{
  //get the current speed of each motor from the encoder ticks
  long new_left, new_right;
  new_left = encoderLeft.read();
  new_right = encoderRight.read();

  encoders.ticks[0] = new_left;
  encoders.ticks[1] = new_right;
  pub_encoders.publish(&encoders);

  // TODO publish low level velocities and compare with high level
}

void write()
{
  // https://www.arduino.cc/reference/en/language/functions/math/map/
  // map(value, fromLow, fromHigh, toLow, toHigh)
  // Map angular wheel joint velocity to motor cmd
  // TODO get rosparam linear max_velocity and convert to max rotational max velocity
  int motor_cmd_left = map(g_wheel_cmd_velocity_left, -g_max_angular_velocity, g_max_angular_velocity, -255, 255);

  // TODO compute PID output
  //the value sent to the motor driver is calculated by the PID based on the error between commanded RPM vs measured RPM
  //the calculated PID ouput value is is capped at -/+ MAX_RPM to prevent the PID from having too much error
  // float measured_angular_velocity_left = encoderLeft.getRPM();
  //motor_pid_left.compute(g_wheel_cmd_velocity_left, measured_angular_velocity_left));
  p_motor_controller_left->setSpeed(motor_cmd_left);

  int motor_cmd_right = map(g_wheel_cmd_velocity_right, -g_max_angular_velocity, g_max_angular_velocity, -255, 255);
  p_motor_controller_right->setSpeed(motor_cmd_right);
}

void eStop()
{
    g_wheel_cmd_velocity_left = 0;
    g_wheel_cmd_velocity_right = 0;
}

void printDebug()
{

}