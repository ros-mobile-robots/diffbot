

/// Encoder pins
#define ENCODER_LEFT_H1 5
#define ENCODER_LEFT_H2 6
// Encoder resolution used for initialization 
// will be read from parameter server
#define ENCODER_RESOLUTION 542

#define ENCODER_RIGHT_H1 7
#define ENCODER_RIGHT_H2 8

/// Motor i2c address
#define MOTOR_DRIVER_ADDR 0x60
#define MOTOR_LEFT 3
#define MOTOR_RIGHT 4

#define K_P 0.6 // P constant
#define K_I 0.3 // I constant
#define K_D 0.5 // D constant

#define PWM_BITS 8  // PWM Resolution of the microcontroller


#define PUBLISH_RATE_COMMAND 20
#define PUBLISH_RATE_IMU 1
#define PUBLISH_RATE_DEBUG 5



#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX