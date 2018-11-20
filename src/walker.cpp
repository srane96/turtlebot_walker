/**
 * BSD 3-Clause License
 * @copyright (c) 2018, Siddhesh Rane
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * @file    walker.cpp
 * @author  Siddhesh Rane
 * @version 1.0
 * @brief walker class implementation;
 *
 * @section DESCRIPTION
 *
 * C++ implementation for Walker class which adds functionality of obstacle
 * avoidance to the gazebo turtlebot.
 */
#include <iostream>
#include "walker.h"
/// Implementation of default Walker constructor
Walker::Walker() {
  /// Default max speed value
  maxSpeed = 0.5;
  /// Default rotation 90deg
  maxRotation = 1.57;
  /// Default safe distance
  safeDistance = 2;
  /// Initially assume obstacle free enviroment
  obstacleDetected = false;
  /// Publisher to publish messages on /navi topic
  pubVel = nh.advertise < geometry_msgs::Twist
      > ("/cmd_vel_mux/input/navi",
    1000);
  /// Subscribe to /scan topic where we get obstacle distance
  subSensor = nh.subscribe < sensor_msgs::LaserScan
      > ("/scan", 500,
  &Walker::sensorCallback, this);
  ROS_INFO("Default ROBOT Created!");
  /// Reset bot velocities to zero
  resetBot();
}
/// Implementation of custom Walker constructor
Walker::Walker(const float& linear, const float& rotation,
               const double& safe) {
  maxSpeed = linear;
  if (rotation > 1.57 or rotation < -1.57) {
    ROS_WARN("Angle should not be more than 90deg");
    ROS_INFO("Using default value");
    maxRotation = rotation;
  } else {
    maxRotation = rotation;
  }
  maxSpeed = linear;
  safeDistance = safe;
  ROS_INFO("Custom ROBOT Created!");
}
/// Destructor that resets the velocity of the bot
Walker::~Walker() {
  resetBot();
}
/// Return obstacleDetected parameter
bool Walker::getObstacleDetected() {
  return obstacleDetected;
}
/// Start the bot with obstacle avoidance functionality
void Walker::startMotion() {
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    /// If obstacle in safe area stop the robot and rotate
    if (getObstacleDetected()) {
      ROS_INFO("Mayday Mayday!");
      msg.linear.x = 0.0;
      msg.angular.z = maxRotation;
    } else {
      ROS_INFO("Path is clear!");
      msg.angular.z = 0.0;
      msg.linear.x = maxSpeed;
    }
    /// Publish the velocity information
    pubVel.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
/// Callback function for the subscriber
void Walker::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  /// Read float array called ranges which contains obstacle distances
  /// for each angle.
  for (const float &m : msg->ranges) {
    if (m < safeDistance) {
      obstacleDetected = true;
      return;
    }
  }
  obstacleDetected = false;
}
/// Set translational and rotational properties to zero
void Walker::resetBot() {
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  pubVel.publish(msg);
}
/// get maximum linear speed by the bot
float Walker::getMaxSpeed() {
  return maxSpeed;
}
/// Set maximum linear speed by the bot
void Walker::setMaxSpeed(const float& speed) {
  maxSpeed = speed;
}
/// Get maximum achievable rotation
float Walker::getMaxRotation() {
  return maxRotation;
}
/// Set maximum achievable rotation
void Walker::setMaxRotation(const float& rotation) {
  maxRotation = rotation;
}
/// Get safe distance from obstacle
double Walker::getSafeDistance() {
  return safeDistance;
}
