^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package diffbot_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2022-03-28)
------------------
* fix: missing subscriber initialization (`#54 <https://github.com/ros-mobile-robots/diffbot/issues/54>`_)
* feat: warn if eStop() is called on mcu
* Change base config: switch motor pins
* feat: add callbacks for low level pids on mcu `#54 <https://github.com/ros-mobile-robots/diffbot/issues/54>`_
  For MCU firmware
  - Add subscribers for left and right motor pid controllers using
  diffbot_msgs::PIDStamped custom message
  - Update debug logging message (different formatting)
  - Update PID controller interface (provide proportional\_, integral\_ and
  derivative\_ values)
* fix: missing diffbot namespace in test
* fix: test_encoders.cpp include name
* feat: add documentation link for unity
* Merge pull request `#42 <https://github.com/ros-mobile-robots/diffbot/issues/42>`_ from joeuser846/motor-defines
  Use MOTOR_LEFT/RIGHT from diffbot_base_config.h instead of hardcoding pin numbers
* Fix filenames
* Use MOTOR_LEFT/RIGHT instead of hardcoding pin numbers
* Contributors: Franz Pucher, Joe User

1.0.0 (2021-08-13)
------------------
* clean CMakeLists.txt: remove dependency
* add dependency diffbot_msgs_generate_messages_cpp to build diffbot_msgs before diffbot_base package
* Refactor: Update interface
  - Exchange jointStates between high hw intf and low level base
  controller
  - Add measured_joint_state publisher in base controller
  to let high level hw intf subscribe to it
  - Remove encoder publisher (todo consider publishing at low rate)
  - Update docstrings of BaseController
  - Extend interface of diffbot::Encoder class to read jointStates
* refactor: renamed methods due to interface change: jointStates
* use new jointState intf between high and low level base
* make use of diffbot namespace
* Update base_controller documentation
  - Update BaseController class
  - Add E_STOP_COMMAND_RECEIVED_DURATION define in diffbot_base_config
* update diffbot_base
  - add diffbot namespace to base_controller library classes
  - update docstrings
  - run rosdoc_lite
  - fix some rosdoc_lite warnings
* add diffbot_base launch arg to choose robot model (for remo and diffbot)
* add AngularVelocitiesStamped.h for rosserial
* remove measured_vel\_ publisher in rosserial
* add low level pid
  - Add pid lib for arduino code
  - make use of pid in base_controller.h
  - update debug logging
  - use ros::time in encoder lib
* set motor_constant=1 (gain trim model)
* make use of new diffbot_msgs in hardware interface and base_controller
* feat: add debug parameter to parameter server
  add option to enable/disable debug logging of the hw interface
* refactor base_controller: use ros time instead of millis
* refactor main.cpp: add base_controller class
  - add base_controller.h with BaseController template class
  - use class publisher and subscriber
  - use BaseController instead of global variables
  - read parameters from parameter server in BaseController
* add encoder resolution to config header and getter/setter
* add include guards to encoder.h
* add base_controller rosserial code using rates
* fix vector assignment of command velocity in hw intf
* add tests for motor controllers
* add compiled rosserial diffbot_msgs
* add adafruit_feather_wing motor controller class header
* moved adafruit_feather_wing.cpp to subfolder
* add adafruit_feather_wing motor controller class
* add author comment
* add abstract MotorControllerInterface class
* remove test_motors.cpp
* update encoder lib for rosserial
  - add doc strings to encoder.h
  - add ticksToAngle method
  - add angularVelocity method
* feature: update diffbot_hw_interface
  - add angular wheel joint velocity publisher
  - add new WheelCmd.msg in diffbot_msgs
  - load new hardware related parameters from
  diffbot_base/config/base.yaml
  - get hardware related parameters from parameter server
  in diffbot_hw_interface
  - add gain trim parameters to dynamic reconfigure cfg
* refactor diffbot_msgs
  - rename Encoder.msg  to Encoders.msg
  - update diffbot_pase includes and method signatures
  - update arduino script
  - change diffbot_msgs license to BSDv3
* add base_controller mcu scripts including tests
* Update package.xml
  change license and add rosserial to exec dependencies
* Contributors: Franz Pucher

0.0.2 (2021-04-30)
------------------
* fix `#30 <https://github.com/fjp/diffbot/issues/30>`_: reset encoders after each new launch of hardware interface
* call reset in encoders setup()
* add method to check if encoder ticks are published
* update logging in encoders.ino
* initialize ``encoder_ticks_`` in hw interface
  initialize values in ``encoder_ticks_`` member array of hardware interface
  to zero because receiving meaningful tick values from the microcontroller
  might take some time. Alleviates problems in `#30 <https://github.com/fjp/diffbot/issues/30>`_
* fix zero time period at control loop start (`#30 <https://github.com/fjp/diffbot/issues/30>`_)
* Contributors: Franz Pucher

0.0.1 (2020-12-22)
------------------
* update diffbot_base to use Encoder msg
* set control loop rate to 10 Hz
* add diffbot_msgs as dependency to diffbot_base
* use queue_size of 10
* update info output in write() method
* update initial fpid parameters and adjust their range
* update info messages with individual pid errors
* update diffbot_base/launch file to use new diffbot.urdf.xacro
* add dynamic reconfigure to pid class for f,p,i,d,i_max/min and antiwindup
* add pid i_max/min and antiwindup to Parameters.cfg
* add boost to CMakeLists.txt
* update CMakeLists.txt using dynamic reconfigure
* add dynamic_reconfigure to package.xml
* add Parameters.cfg for feed forward gain
* reset pid error and cmd values in case setpoint and p_error is zero: avoids non-vanishing i_error
* tune pid and feed forward gains
* fix major bug: prev_time not updated
* refactor pid to enable dynamic reconfigure for both motors
* add debug output logging messages to pid class and enable dynamic reconfigure - needs param for more than one pid
* let PID inherit from control_toolbox::Pid
* add pid controller for motors from ros control toolbox 
* add pid.cpp in cmake
* Update version, mail and license
* fix compile error and runtime errors due to dynamic reconfigure using initPid instead of setGains and tune PID values (P=10.0)
* Update README.md and add comments to code
* move packages from ros/src to repository toplevel folder
* Contributors: Franz Pucher
