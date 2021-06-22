# Ros & Robot Arm Package Installation and Configuration
## Table of contents
* [Task Info](#task-info)
* [Ros Installation](#Ros-Installation)
* [Create Work Space ](#Create-Work-Space )
* [Robot Arm Package Installation ](#Robot-Arm-Package-Installation )
* [Publisher Error Handling](#Publisher-Error-Handling )

## Task Info
This documentation shows the steps of [Ros Noetic](http://wiki.ros.org/noetic/Installation) and [Robot Arm Package](https://github.com/smart-methods/arduino_robot_arm_gripper) installation and configuration. The task includes many subtasks, which are installing Robot Operating System correctly. Then create a catkin workspace that you will use for download packages and libraries into it. After that, you can download the robot arm package and run simulators such as Rviz or Gazebo that will help to show how the arm interacts in the specific environment. In addition, you will almost face some errors, especially if you are using Ros Noetic version.


## Ros Installation
To install Ros, you should have [Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu) or [Debian](http://wiki.ros.org/noetic/Installation/Debian) on your machine. Then, you can open the [Ros Noetic installation wiki](http://wiki.ros.org/noetic/Installation/Ubuntu) and follow the command steps.

I will simplify the process with as few steps as possible.

1- Open the Terminal and write the next commands sequentially.

2- Setup your sources for giving installation Ros permission to your System.

``(url)sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'``

3- Set up your keys.

``curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -``

Note: you may need to install curl tool using this command: ``sudo apt install curl``.

4- Make sure the Advance Package Tools is up to date use the command ``sudo apt update``.

5-Install Ros.

``sudo apt install ros-noetic-desktop-full``

6- Setup the environment by using the source command with the script /opt/ros/noetic/setup.bash, It can be convenient to automatically source this script every time a new shell is launched. These commands will do that for you.

``echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc``

``source ~/.bashrc``

To test the Ros, try to run the command ``roscore``; this will create a master node, the master node will make later communication with other nodes.


<img src="https://user-images.githubusercontent.com/86131920/122996010-03b91500-d3b3-11eb-96db-bb730bc9f5db.png" width=70% height=70%>


## Create Work Space

A catkin workspace is a directory (folder) in which you can create or modify existing catkin packages. The catkin structure simplifies the build and installation process for your ROS packages.

1- Before create and build a catkin workspace, you should source your environment by applying the command.

``$ source /opt/ros/noetic/setup.bash.``

2- Create a catkin workspace.

The next command creates a directory for your workspace and the catkin_make command is a convenience tool for working with catkin workspaces. Running it the first time in your workspace, it will create a CMakeLists.txt link in your 'src' folder.

``$ mkdir -p ~/catkin_ws/src``

``$ cd ~/catkin_ws/``

``$ catkin_make``

Your catkin workspace will look like this :

<img src="https://user-images.githubusercontent.com/86131920/122997623-df5e3800-d3b4-11eb-82c6-a4d913da64e3.png" width=90% height=90%>



