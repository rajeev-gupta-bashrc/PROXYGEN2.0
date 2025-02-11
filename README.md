# PROXYGEN2.0
Robotic arm in ros2 humble using moveit2. To use this package, make sure you've installed ros2 humble and gazebo harmonic from the official source.

[ROS2_Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

[Gazebo_Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/)


## Clone this repo into your worksapce, if you don't have a ros workspace yet, create one:

```
cd ~
mkdir -p robotic_arm_ws/src
cd robotic_arm_ws/src
```
- or in your custom workspace src folder, clone this repo:

```
git clone https://github.com/rajeev-gupta-bashrc/PROXYGEN2.0.git
cd ..
colcon build
```

- add the source command in .bashrc file:
```
echo 'source ~/robotic_arm_ws/install/setup.bash' >> ~/.bashrc
```

## Visualize the Robot:
- install urdf-tutorial package

```
sudo apt-get install ros-humble-urdf-tutorial
```

- run this code in a new terminal
```
ros2 launch urdf_tutorial display.launch.py model:=$HOME/robotic_arm_ws/src/PROXYGEN2.0/proxygen_description/xacro/proxygen.full.xacro
```
![Robot in Action](https://github.com/rajeev-gupta-bashrc/PROXYGEN2.0/blob/main/images/arm_rviz.png)

- You'll also see a small window of sliders which can be used to change the angles of the robotic arm joints. 


## Launch robot in Gazebo:

```
ros2 launch proxygen_description gazebo.launch.py 
```

![Robot in Action](https://github.com/rajeev-gupta-bashrc/PROXYGEN2.0/blob/main/images/arm_gazebo.png)

- Note: Gazebo will launch in paused mode by default. If you don't want to launch it in paused mode, run this:
```
ros2 launch proxygen_description gazebo.launch.py paused:=False
```












## Ref:
[How to Simulate a Robotic Arm in Gazebo – ROS 2](https://automaticaddison.com/how-to-simulate-a-robotic-arm-in-gazebo-ros-2/)

[TurtleBot3 Manipulation](https://github.com/ROBOTIS-GIT/turtlebot3_manipulation)
