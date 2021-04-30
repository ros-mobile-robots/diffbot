^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package diffbot_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Update diffbot.launch
* fix local costmap configuration issue
* Contributors: Franz, Franz Pucher

0.0.1 (2020-12-22)
------------------
* increase goal reached tolerance
* include diffbot/laserscan in rviz
* add navigation launch file for real robot
* add office map
* remove diffbot_control.launch
  Since the diffbot.launch at diffbot_gazebo already launch the controllers.
  It can me removed.
* Correct local planner name parameter.
* Update dwa_local_planner_params.yaml
* Update base_local_planner_params.yaml
* Update move_base.launch
* add missing diffbot_navigation.rviz
* add world_name arg to navigation diffbot.lauch
* add launch files, configs and map to diffbot_navigation
* add initial diffbot_navigation package
* Contributors: Franz Pucher, pxalcantara
