# UR10-Robotiq2F140
## Desciption
This project aims to combine the Universal Robot and Robotiq together with ROS. The versions of robot arm and gripper are UR10 and Robotiq 2F-140 Grippers.
However, robotiq doesn't have an updated version ROS dependencies in ROS wiki which cause failure in building workspace. 
## Prerequisite
You need to download UR-Driver and UR-Description which are necessary for robot arm driving, motion planning and visualization.
The URL sources:

UR Driver: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver

UR Decription: https://github.com/ros-industrial/universal_robot

You can also install them in ROS by:

```sudo apt-get install ros-noetic-ur-driver```

```sudo apt-get install ros-noetic-ur10-moveit-config```

Robotiq has a formal package provided in ROS wiki: http://wiki.ros.org/robotiq. But the highest distros is kinetic. When build this package in noetic workspace, the problems appear:

When I follow the tutorial of robotiq 2F gripper (http://wiki.ros.org/robotiq/Tutorials/Control%20of%20a%202-Finger%20Gripper%20using%20the%20Modbus%20RTU%20protocol%20%28ros%20kinetic%20and%20newer%20releases%29), I tried **rosdep install** the dependencies the package needed. But the distros versions of **python3-modbus** are imcompatitable. 

My solution is ignore this step and install a package which can provide modbus functions individually. You can download it from:

https://github.com/BoyangZhangFromHKUST/UR10-Robotiq2F140/tree/main/modbus

The control method of robotiq is from s-nam: https://github.com/s-nam/control_2f_gripper_and_ur10

## Control Strategy
Connect UR10 to your PC with a wireline, set up the ip address and establish a two-way communication.

Connect Robotiq gripper to your PC with a USB convert line, and do not forget to give it a 24V power.

UR10:

```roslaunch ur_robot_driver ur10_bringup.launch robot_ip:=192.168.1.102```

```roslaunch ur10_moveit_config motion_planning_execution.launch```

```roslaunch ur10_moveit_config moveit_rviz.launch```

Robotiq:

```roscore```

```rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0```

```rosrun control_2f_gripper_and_ur10 gripper_test.py```

Then you can control robot arm and robotiq 2F gripper respectively.

## Summary
In the first stage, we just attempt to find and solve problems in the process of using ROS noetic. In next stage, we will try to control the arm and gripper together. In the third stage, we will try to integrate them into a launch file. In the fourth stage, we will try using ROS2 control them!
