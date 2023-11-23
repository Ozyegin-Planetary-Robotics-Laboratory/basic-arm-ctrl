//
// Created by erayeminocak on 10/18/23.
//

#ifndef ARM_BASIC_CTRL_MAIN_H
#define ARM_BASIC_CTRL_MAIN_H

#include <memory>

#include <moveit/move_group_interface/move_group_interface.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#define BUFFER_SIZE 128
#define BUILDER_SIZE 24

void Log(const char *msg);
void Log2V(const char *c1, char *c2);
void Log4VT(const char *c1, char *c2, char *c3, char *c4);
void PrintValue(double d);

#endif //ARM_BASIC_CTRL_MAIN_H