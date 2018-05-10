# Turtlebot 3 DAGGER #

Turtlebot 3 DAGGER main task is to train a neural network with the imitation learning Dagger algorithm. It uses the turtlebot_3_autorace as master policy and run in a gazebo simulated environment.

## Getting Started ##

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
