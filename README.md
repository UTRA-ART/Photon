# Useful links for this branch #

https://answers.ros.org/question/252478/how-to-build-a-package-from-source-on-ubuntu-mate-1604-lts-and-ros-kinetic/

http://wiki.ros.org/ackermann_steering_controller

http://wiki.ros.org/steer_bot_hardware_gazebo

http://wiki.ros.org/steer_drive_controller

https://github.com/CIR-KIT-Unit03/cirkit_unit03_simulator/tree/master/cirkit_unit03_gazebo

https://github.com/CIR-KIT/steer_drive_ros






# Photon on ROS #

This project runs on ROS melodic for Ubuntu 18.04 LTS. Photon is a robot being built to compete in IARRC.

## Setting up the ROS Environment ##

### Install Ubuntu 18.04 LTS ###
This is dependent on what OS and computer is currently used.

### Install ROS Melodic ###
The following instructions are taken from the ROS wiki [install melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) page:
```bash
# Set up sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Set up keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Melodic Desktop-Full Install
sudo apt install ros-melodic-desktop-full

# Environment setup
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Dependencies for buiding packages
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update
```

### Install the Navigation Package ###
Provides the [Navigation Stack](http://wiki.ros.org/navigation) package which is used for autonomous navigation.
```bash
sudo apt-get install ros-melodic-navigation
```

### Install Robot Localization ###
Provides the [Robot Localization](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html) package which is used for localizing the robot.
```bash
sudo apt-get install ros-melodic-robot-localization
```

## Cloning this repository ##

```bash
# Make workspace
mkdir -p photon-ws/src

# Clone the repository
git clone https://github.com/UTRA-ART/Photon.git

# Clone steer drive plugin repo
cd src/
git clone https://github.com/tsedl/steer_drive_ros.git
cd steer_drive_ros
git checkout melodic-devel

# Check dependencies
rosdep check --from-paths src --ignore-src --rosdistro melodic

# Install dependencies
rosdep install --from-paths src --ignore-src --rosdistro melodic -y

# Build
catkin_make
```
After, clone this repository into the `/src` folder.
> **Optionally:** clone this repository as your source folder, i.e. rename `Photon/` to `src/`

---
<p align="center">
<img src="https://raw.githubusercontent.com/UTRA-ART/SLAM/dev/docs/res/utra-logo.png" alt="UTRA logo" width="200"/>
</p>
<p align = "center"><b>University of Toronto Robotics Association</b></p>
<p align = "center">Autonomous Rover Team</p>
