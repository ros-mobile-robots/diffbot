#include "adafruit_feather_wing/adafruit_feather_wing.h"


diffbot::AdafruitMotorController::AdafruitMotorController(uint8_t motor_num, uint8_t addr)
{
    motor_driver_ = Adafruit_MotorShield(addr);
    // Select which 'port' (motor pin) M1, M2, M3 or M4.
    pMotor_ = motor_driver_.getMotor(motor_num);
}

void diffbot::AdafruitMotorController::begin(uint16_t freq)
{
    motor_driver_.begin(freq);
}

void diffbot::AdafruitMotorController::setSpeed(int value)
{
    if (value > 0)
    {
        pMotor_->run(FORWARD);
    }
    else if (value < 0)
    {
        pMotor_->run(BACKWARD);
        // AdafruAdafruit_MotorShield requires uint8 values from 0 to 255
        // Make sure to convert the negative value to a positive one
        value = value * -1;
    }
    else // zero speed
    {
        // Cut power to the motor
        pMotor_->run(RELEASE);
    }

    pMotor_->setSpeed(value);
}