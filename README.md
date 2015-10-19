# Ros-Test-Example
A ROS Workspace containing an example car simulation to show GTest and Rostest

Contains full workspace

Dependancies:
 -  roscpp
 -  roscore
 -  rviz
 -  nav_msgs
 -  std_msgs
 -  roslaunch
 -  rostest
 -  gtest

How-to-build:
  catkin_make

How-to-run:
  - source devel/setup.bash
  - Simulation: roslaunch cars run.launch
    - also launches an example car called BEG_IN_001
  - Car       : roslaunch cars car.launch numberPlate:=<Identifier>
    - no two cars may have the same identifier

How-to-test:
  - Full integration test: 
    - rostest cars simHz.test
    - rostest cars carHz.test
  - Unit tests:
    - start roscore
    - catkin_make run_tests
