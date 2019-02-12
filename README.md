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
```cmd
cd catkin_ws/src
git clone https://github.com/ros-industrial/ur_modern_driver
cd ..
catkin_make
source devel/setup.bash
```
The drivers are not working probaly after installation. We need to modify the hardware interface of drivers to make it works with ROS Kinetic.
```cmd
cd catkin_ws/src/ur_modern_driver/src
gedit ur_hardware_interface.cpp
```
Then replacing all the contains in "ur_hardware_interface.cpp" by the code in this [link](https://github.com/iron-ox/ur_modern_driver/blob/883070d0b6c0c32b78bb1ca7155b8f3a1ead416c/src/ur_hardware_interface.cpp)
Save cahnges and continue.

```cmd
cd
cd catkin_ws
catkin_make
source devel/setup.bash
```