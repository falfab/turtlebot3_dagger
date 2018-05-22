# Turtlebot 3 DAGGER #

Turtlebot 3 DAGGER main task is to train a neural network with the imitation learning Dagger algorithm. It uses the turtlebot_3_autorace as master policy and run in a gazebo simulated environment.

## Getting Started ##

Install dependent packages for TurtleBot3 control.
```shell
sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
```

```bash
cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_autorace.git
cd ~/catkin_ws && catkin_make
```


After you cloned the repository into your ROS workspace you need do the following shell command to move the track model into the gazebo models directory:

```shell
mv ./world/turtlebot3_track $HOME/.gazebo/models/
```

Install dependencies, compile and run.

## Autorace ##

Run in three different terminals the following command:

- Launch the turtlebot fake controller

```shell
roslaunch turtlebot3_dagger turtlebot3_fake_controller.launch
```

- Launch the turtlebot gazebo environment

```shell
roslaunch turtlebot3_dagger turtlebot3_gazebo.launch
```

- Launch the master policy (project and compensate the image, detect the lane, publish command)

```shell
roslaunch turtlebot3_dagger turtlebot3_master.launch
```
