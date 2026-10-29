#ifndef MOTOR_CONTROLLER_INTF_H
#define MOTOR_CONTROLLER_INTF_H

/*
 * Author: Franz Pucher
 */

namespace diffbot {

    /** \brief Abstract base interface class for a motor controller
     * 
     * Inherit from this base class and specify the type of 
     * motor driver. The interface provides \ref setSpeed, which
     * is an abstract method and must therefore be implemented.
     */
    template<typename TMotorDriver>
    class MotorControllerIntf
    {
        public:

            /** \brief Set the speed of the a motor
             * 
             * Implement this method to set the speed of a 
             * motor that is connected to the \ref motor_driver_
             * which is of type \ref TMotorDriver.
             * 
             * \param value positive or negative value to set the direction
             * and speed of the motor.
             * 
             */
            virtual void setSpeed(int value) = 0;

        protected:
            // Generic motor driver
            TMotorDriver motor_driver_;
    };
}

#endif // MOTOR_CONTROLLER_INTF_H