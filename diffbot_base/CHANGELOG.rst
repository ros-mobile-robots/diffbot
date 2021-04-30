^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package diffbot_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
