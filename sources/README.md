# RobotDog_v2

Robodog sources are compiled as a part of ROS build system.

Steps to install and setup:<br/>
1) Install ROS (Kinetic/Melodic) to ubuntu on Desktop PC or Raspberry Pi:<br/>
  http://wiki.ros.org/ROS/Installation<br/>
  http://wiki.ros.org/Installation/Ubuntu<br/>
2) Create ROS Workspace:<br/>
  http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment<br/>
3) Clone repositories:<br/>
  > cd ~/catkin_ws/src<br/>
  > git clone https://github.com/wjwwood/serial.git<br/>
  > git clone https://github.com/RoboLabHub/RoboDog_v2<br/>
4) Fix problem with USB on ubuntu:<br/>
  > cd ~/catkin_ws/src/RoboDog_v2/sources/Misc<br/>
  > sudo cp 99-usb.rules /etc/udev/rules.d/<br/>
  > sudo udevadm control --reload-rules<br/>
  > sudo udevadm trigger<br/>
5) Compile sources:<br/>
  > cd ~/catkin_ws<br/>
  > catkin_make<br/>
  > source ~/catkin_ws/devel/setup.bash<br/>
6) Run robot:<br/>
  > cd ~/catkin_ws<br/>
  > rosrun robodog_v2 robodog_v2_hw<br/>

Robot controlled by native G-Code controller, but before running it all servos should be tuned.<br/>
Each servo has its own offset for fine tuning, for example, for Leg1 (that contains 3 servos) the offsets are setup here:<br/>
https://github.com/RoboLabHub/RoboDog_v2/blob/master/sources/Services/RobotHW.cpp#L47
