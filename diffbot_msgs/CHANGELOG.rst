^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package diffbot_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2022-03-28)
------------------

1.0.0 (2021-08-13)
------------------
* add AngularVelocitiesStamped.msg
* add and refactor diffbot_msgs
  - Rename WheelCmd.msg to WheelsCmd.msg and removed Header
  - Add WheelsCmdStamped.msg
  - PID.msg and PIDStamped.msg
  - Refactor Encoders.msg (removing Header)  and add EncodersStamped.msg
* add WheelCmd.msg for angular wheel joint velocities
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
* Contributors: Franz Pucher

0.0.2 (2021-04-30)
------------------

0.0.1 (2020-12-22)
------------------
* Initial release
* Create diffbot_msgs package
* Add encoder message
* Update documentation
* Contributors: Franz Pucher