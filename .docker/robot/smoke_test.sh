#!/bin/bash
# Smoke test of the robot image, without hardware (CI runs it, see .github/workflows/robot_container.yml):
# the packages are built, roscore starts, the robot models expand, the launch files parse, and the
# programs that talk to the hardware are there. It can't show that the Teensy or the lidar work.
# Run it in the image:
#   docker run --rm -v "$PWD/.docker/robot/smoke_test.sh:/smoke_test.sh:ro" diffbot-robot:noetic bash /smoke_test.sh
set -eo pipefail

echo "== The packages are found"
for p in $DIFFBOT_PACKAGES rosserial_python teleop_twist_keyboard; do
    rospack find "$p"
done

echo "== The programs for the hardware exist and are executable"
# catkin_find looks them up as rosrun and roslaunch do
for exe in "diffbot_base diffbot_base" "rplidar_ros rplidarNode" "rosserial_python serial_node.py"; do
    path=$(catkin_find --libexec --first-only $exe)
    test -x "$path"
    echo "$path"
done

echo "== Both robot models expand"
for model in remo diffbot; do
    xacro "$(rospack find ${model}_description)/urdf/${model}.urdf.xacro" > /tmp/$model.urdf
    echo "$model: $(grep -c '<link ' /tmp/$model.urdf) links"
done

echo "== roscore starts"
roscore > /tmp/roscore.log 2>&1 &
for i in $(seq 1 30); do rosparam get /rosdistro > /dev/null 2>&1 && break; sleep 1; done
rosparam get /rosdistro

echo "== The launch files parse"
for model in remo diffbot; do
    roslaunch --nodes diffbot_bringup bringup.launch model:=$model
done
# The lidar driver in the robot's namespace, so its scans are on /diffbot/scan
roslaunch --nodes diffbot_bringup lidar.launch | grep -x /diffbot/rplidarNode

echo "== Remo's bringup starts without hardware: the model loads and both controllers run"
# Without the Teensy, rosserial can't open its port and the hardware interface gets no wheel
# feedback ("No measured joint states publishers"); the controllers start anyway.
roslaunch diffbot_bringup bringup.launch model:=remo > /tmp/bringup.log 2>&1 &
running=0
for i in $(seq 1 60); do
    running=$(rosservice call /diffbot/controller_manager/list_controllers 2>/dev/null | grep -c 'state: "running"' || true)
    [ "$running" -eq 2 ] && break
    sleep 1
done
echo "running controllers: $running"
[ "$running" -eq 2 ] || { cat /tmp/bringup.log; exit 1; }
rostopic list | grep -x /diffbot/mobile_base_controller/odom
rosparam get /diffbot/robot_description | grep -c '<link ' > /dev/null

kill %2 %1
echo "Smoke test passed"
