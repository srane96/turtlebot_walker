#include <iostream>
#include "walker.h"
Walker::Walker() {
  maxSpeed = 0.5;
  maxRotation = 1.57;
  safeDistance = 2;
  obstacleDetected = false;
  pubVel = nh.advertise < geometry_msgs::Twist
      > ("/cmd_vel_mux/input/navi",
    1000);
  subSensor = nh.subscribe < sensor_msgs::LaserScan
      > ("/scan", 500,
  &Walker::sensorCallback, this);
  ROS_INFO("Default ROBOT Created!");
  resetBot();
}
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
Walker::~Walker() {
  resetBot();
}
bool Walker::getObstacleDetected() {
  return obstacleDetected;
}
void Walker::startMotion() {
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    if (getObstacleDetected()) {
      ROS_INFO("Mayday Mayday!");
      msg.linear.x = 0.0;
      msg.angular.z = maxRotation;
    } else {
      ROS_INFO("Path is clear!");
      msg.angular.z = 0.0;
      msg.linear.x = maxSpeed;
    }
    pubVel.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
void Walker::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  for (const float &m : msg->ranges) {
    if (m < safeDistance) {
      obstacleDetected = true;
      return;
    }
  }
  obstacleDetected = false;
}
void Walker::resetBot() {
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  pubVel.publish(msg);
}
float Walker::getMaxSpeed() {
  return maxSpeed;
}
void Walker::setMaxSpeed(const float& speed) {
  maxSpeed = speed;
}
float Walker::getMaxRotation() {
  return maxRotation;
}
void Walker::setMaxRotation(const float& rotation) {
  maxRotation = rotation;
}
double Walker::getSafeDistance() {
  return safeDistance;
}
