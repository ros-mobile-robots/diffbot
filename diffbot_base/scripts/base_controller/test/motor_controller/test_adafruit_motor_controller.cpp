/*
 * Author: Franz Pucher
 */

// Include unity library
// For details see https://docs.platformio.org/en/latest/plus/unit-testing.html#api
#include <unity.h>

#include <adafruit_feather_wing/adafruit_feather_wing.h>

using namespace diffbot;

AdafruitMotorController AMC = AdafruitMotorController(3);
MotorControllerIntf<Adafruit_MotorShield>* pMotorController = &AMC;


void motorSweep(MotorControllerIntf<Adafruit_MotorShield>* i_pMotorController)
{
  int i;
  for (i=0; i<255; i++) {
    i_pMotorController->setSpeed(i);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    i_pMotorController->setSpeed(i);  
    delay(10);
  }
  
  Serial.print("tock");

  for (i=0; i > -255; i--) {
    i_pMotorController->setSpeed(i);  
    delay(10);
  }
  for (i=-255; i!=0; i++) {
    i_pMotorController->setSpeed(i);  
    delay(10);
  }

  Serial.print("tech");
  i_pMotorController->setSpeed(0);
  delay(1000);
}


void setup() {
  // NOTE!!! Wait for >2 secs
  // if board doesn't support software reset via Serial.DTR/RTS
  delay(2000);
  UNITY_BEGIN();

  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("AdafruitMotorController Test");

  AMC.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
}

uint8_t cycle = 0;
uint8_t max_loop_cycles = 1;

void loop() {

  if (cycle < max_loop_cycles)
  {
    Serial.print("tick");

    motorSweep(pMotorController);
  }
  else
  {
    pMotorController->setSpeed(0);
    delay(1000);
    UNITY_END();
  }

  cycle++;
}