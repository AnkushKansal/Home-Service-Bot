# Home-Service-Bot

In this project, a mobile robot is asked to navigate to a pick-up zone for picking 
up a green cube. After that, it moves while carrying the cube to the drop-off
zone position and drops the cube there.

<img src="https://github.com/AnkushKansal/Home-Service-Bot/blob/master/TowardsDestination.PNG"  width="700" height="400" />

## How it works
The mobile robot first drives around and scan the house using laser for generating 
a static map about this place. Having the map, it uses odometry and laser data 
to localize itself with adaptive monte carlo localization (AMCL). Upon receiving 
a navigation goal, it plans forward the trajectory using Dijkstra's algorithm and 
navigate to the goal.

## Description
The project consists of the following parts:
1. A Gazebo world and a mobile robot.
2. ROS packages: [map_server](http://wiki.ros.org/map_server), 
[amcl](http://wiki.ros.org/amcl), [move_base](http://wiki.ros.org/move_base),
[slam-gmapping](http://wiki.ros.org/slam_gmapping) 
and [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard).

## Prerequisites
1. ROS (Melodic or Kinetic), Gazebo on Linux
2. CMake & g++/gcc, C++11
3. Install xterm `sudo apt-get install xterm`
4. Python with pip `sudo apt-get install python-pip`
5. Install some dependencies
```
$ sudo apt-get update && sudo apt-get upgrade -y
$ sudo apt-get install ros-${ROS_DISTRO}-map-server
$ sudo apt-get install ros-${ROS_DISTRO}-amcl
$ sudo apt-get install ros-${ROS_DISTRO}-move-base
$ sudo apt-get install ros-${ROS_DISTRO}-slam-gmapping
$ pip install rospkg
```

## Build and Launch

1. Clone the project and initialize a catkin workspace
```
$ mkdir -p catkin_ws/src && cd ./catkin_ws/src
$ catkin_init_workspace
$ https://github.com/AnkushKansal/Home-Service-Bot.git
```

2. Navigate back to the `catkin_ws`folder and build the project
```
$ cd ..
$ catkin_make
```

3. Run available scripts to launch
```
$ source devel/setup.bash
$ chmod u+x ./src/scripts/home_service.sh
$ ./src/scripts/home_service.sh
```
Note: To redraw the map, close all ros terminals and re-run the script 
`test_slam.sh` instead. While all terminal is open, run 
`rosrun map_server map_saver -f MAP_NAME`

### Part 1: SLAM

The first thing the robot can do is simultaneous localization and mapping (SLAM). To perform SLAM, run the `test_slam.sh`script:

```shell
$ ./test_slam.sh
```

Several windows will automatically open (this may take a few seconds). You will see the robot in Rviz in what looks like a fairly blank map. To operate the robot, click on the window for the `keyboard_teleop` node, and follow the commands there. As the robot moves around the world, the map will begin to appear in Rviz.

  ##### Package used : [slam-gmapping](http://wiki.ros.org/slam_gmapping). 
  This package is responsible for localizing the bot and mapping the environment simultaneously. It uses AMCL algorithm along with occupancy grid mapping algorithm to map.

### Part 2: Navigation

The next task for the robot is navigation. To test the robot's navigation capabilities, run the `test_navigation.sh` script:

```shell
$ ./test_navigation.sh
```

Again, several windows will open (this may take a few seconds). This time you'll see the robot in a completed map in Rviz. Click the "2D Nav Goal" button and click/drag somewhere on the map to command the robot. The robot will find a path to the goal location and follow it.

Navigation is performed by Dijkstra Algorithm which is a variant of uniform path cost search method.

### Part 3: Full Service

Now that the world is mapped and the robot can follow commands, the robot can be instructed to pick up and drop off a simulated object at different waypoints. To do this, run the `home_service.sh` script:

```shell
$ ./home_service.sh
```

An item (represented by a green cube) will show up in Rviz. The robot will navigate to the item, at which point it will disappear (indicating it has been picked up), and then the robot will navigate to another point and drop off the item, at which point the item will reappear.

### Overall packages used
The ros-perception/slam_gmapping package performs laser based SLAM, Simultaneous Location And Mapping. A 2D occupancy grid map is created using the laser data and turtlebot robot (turtlebot package) pose data as it moves through an environment, which the user can do using the keyboard_teleop package. From a given map created by the SLAM package, the AMCL package is used to localise the robot within the environment using a adaptive particle filter. The navigation package can be given a goal and will use the odometry(from AMCL) and other sensor information to navigate to a given goal. As usual the RVIZ package is used to visualise everything.
