^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package diffbot_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2022-03-28)
------------------
* fix robot spawning in world origin using pid gains (`#57 <https://github.com/ros-mobile-robots/diffbot/issues/57>`_)
* include pid.yaml to avoid Gazebo error messages
* Contributors: Franz Pucher

1.0.0 (2021-08-13)
------------------
* use db_world by default for diffbot_control launch file
* Update package.xml
  update license and email
* increase linear and angular velocity limits
* update wheel_seperation parameter
* add model arg to launch files for remo
* Contributors: Franz Pucher

0.0.2 (2021-04-30)
------------------

0.0.1 (2020-12-22)
------------------
* update diffbot control and gazebo launch files: prepare for slam packages
* Create README.md
* update control launch file to use rviz config and corridor world
* update rviz config: use odom as ref frame and show laser scan
* rename caster wheel in controller config
* Update version, mail and license
* move packages from ros/src to repository toplevel folder
* Contributors: Franz Pucher
