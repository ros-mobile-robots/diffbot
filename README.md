# 2wd-robot
Autonomous 2wd robot using ROS on RPi 4 B

## ROS Installation

The robot setup is supposed to run on Ubuntu 18.04 Bionic. [ROS Melodic]() is intended to run with this Ubuntu version.

From the [catkin tutorial](https://wiki.ros.org/catkin/Tutorials) here are the commands used to create the workspace:
Use [`catkin build`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) instead of [`catkin_make`](https://wiki.ros.org/catkin/commands/catkin_make).
[Here is why](https://robotics.stackexchange.com/questions/16604/ros-catkin-make-vs-catkin-build).

```
$ mkdir -p ~/ros/src
$ cd ~/ros/
$ catkin build
``` 

<pre><font color="#8AE234"><b>fjp@ubuntu</b></font>:<font color="#729FCF"><b>~/git/2wd-robot/ros</b></font>$ catkin build
<font color="#75507B">----------------------------------------------------------------</font>
<font color="#06989A">Profile:</font>                     <font color="#C4A000">default</font>
<font color="#06989A">Extending:</font>             <font color="#4E9A06">[env]</font> <font color="#C4A000">/opt/ros/melodic</font>
<font color="#06989A">Workspace:</font>                   <font color="#C4A000">/home/fjp/git/2wd-robot/ros</font>
<font color="#75507B">----------------------------------------------------------------</font>
<font color="#06989A">Build Space:</font>        <font color="#4E9A06">[exists]</font> <font color="#C4A000">/home/fjp/git/2wd-robot/ros/build</font>
<font color="#06989A">Devel Space:</font>        <font color="#4E9A06">[exists]</font> <font color="#C4A000">/home/fjp/git/2wd-robot/ros/devel</font>
<font color="#06989A">Install Space:</font>      <font color="#3465A4">[unused]</font> <font color="#C4A000">/home/fjp/git/2wd-robot/ros/install</font>
<font color="#06989A">Log Space:</font>         <font color="#CC0000">[missing]</font> <font color="#C4A000">/home/fjp/git/2wd-robot/ros/logs</font>
<font color="#06989A">Source Space:</font>       <font color="#4E9A06">[exists]</font> <font color="#C4A000">/home/fjp/git/2wd-robot/ros/src</font>
<font color="#06989A">DESTDIR:</font>            <font color="#3465A4">[unused]</font> <font color="#C4A000">None</font>
<font color="#75507B">----------------------------------------------------------------</font>
<font color="#06989A">Devel Space Layout:</font>          <font color="#C4A000">linked</font>
<font color="#06989A">Install Space Layout:</font>        <font color="#C4A000">None</font>
<font color="#75507B">----------------------------------------------------------------</font>
<font color="#06989A">Additional CMake Args:</font>       <font color="#C4A000">None</font>
<font color="#06989A">Additional Make Args:</font>        <font color="#C4A000">None</font>
<font color="#06989A">Additional catkin Make Args:</font> <font color="#C4A000">None</font>
<font color="#06989A">Internal Make Job Server:</font>    <font color="#C4A000">True</font>
<font color="#06989A">Cache Job Environments:</font>      <font color="#C4A000">False</font>
<font color="#75507B">----------------------------------------------------------------</font>
<font color="#06989A">Whitelisted Packages:</font>        <font color="#C4A000">None</font>
<font color="#06989A">Blacklisted Packages:</font>        <font color="#C4A000">None</font>
<font color="#75507B">----------------------------------------------------------------</font>
<font color="#34E2E2"><b>Workspace configuration appears valid.</b></font>

<font color="#34E2E2"><b>NOTE:</b></font> Forcing CMake to run for each package.
<font color="#75507B">----------------------------------------------------------------</font>
[build] No packages were found in the source space &apos;/home/fjp/git/2wd-robot/ros/src&apos;
[build] No packages to be built.
[build] Package table is up to date.                                                                                                                                                              
Starting  <font color="#8AE234"><b>&gt;&gt;&gt;</b></font> <font color="#34E2E2"><b>catkin_tools_prebuild               </b></font>                                                                                                                                                
<font color="#555753"><b>Finished</b></font>  <font color="#4E9A06">&lt;&lt;&lt;</font> <font color="#06989A">catkin_tools_prebuild               </font> [ <font color="#C4A000">10.0 seconds</font> ]                                                                                                                               
[build] <font color="#8AE234"><i><b>Summary:</b></i></font> <i>All </i><i><b>1</b></i> <i>packages succeeded!</i>                                                                                                                                                        
[build]   <font color="#555753"><i><b>Ignored:   None.</b></i></font>                                                                                                                                                                        
[build]   <font color="#555753"><i><b>Warnings:  None.</b></i></font>                                                                                                                                                                        
[build]   <font color="#555753"><i><b>Abandoned: None.</b></i></font>                                                                                                                                                                        
[build]   <font color="#555753"><i><b>Failed:    None.</b></i></font>                                                                                                                                                                        
[build] <i><b>Runtime:</b></i> <i>10.1 seconds total.</i>                         </pre>


## Create a new Catkin package

To create a new Catkin package when in your workspace use the following command:

```
$ catkin create pkg PKG_NAME
```
For example the `control` package was created with the following command:

<pre><font color="#8AE234"><b>fjp@ubuntu</b></font>:<font color="#729FCF"><b>~/git/2wd-robot/ros/src</b></font>$ catkin create pkg control
Creating package &quot;control&quot; in &quot;/home/fjp/git/2wd-robot/ros/src&quot;...
Created file control/package.xml
Created file control/CMakeLists.txt
Successfully created package files in /home/fjp/git/2wd-robot/ros/src/control.</pre>
