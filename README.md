# Prerequisites
First of all, the project needs these supporting packages to be installed. The instruction assumes Catkin workspace has already installed.
* **ROS-Industrial's universal_robot package**
```cmd
cd catkin_ws/src
git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot.git
cd ..
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro kinetic
catkin_make
source devel/setup.bash
```
* **ROS driver for CB1, CB2 and CB3+ controllers with Universal Robots**
* 