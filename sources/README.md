# RobotDog_v2

Robodog sources are compiled as a part of ROS build system.

Steps to install and setup:<br/>
1) Install ROS (Kinetic/Melodic) to ubuntu on Desktop PC or Raspberry Pi:<br/>
  http://wiki.ros.org/ROS/Installation
  http://wiki.ros.org/Installation/Ubuntu
2) Create ROS Workspace:<br/>
  http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
3) Clone repositories:<br/>
  > cd ~/catkin_ws/src
  > git clone https://github.com/wjwwood/serial.git
  > git clone https://github.com/RoboLabHub/RoboDog_v2
4) Fix problem with USB on ubuntu:<br/>
  > cd ~/catkin_ws/src/RoboDog_v2/sources/Misc
  > sudo cp 99-usb.rules /etc/udev/rules.d/
  > sudo udevadm control --reload-rules
  > sudo udevadm trigger
5) Compile sources:<br/>
  > cd ~/catkin_ws
  > catkin_make
6) Run robot:<br/>
  > cd ~/catkin_ws
  > rosrun robodog_v2 robodog_v2_hw
