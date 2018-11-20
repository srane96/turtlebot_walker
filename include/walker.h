#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
class Walker {
 private:
  bool obstacleDetected;
  geometry_msgs::Twist msg;
  ros::Publisher pubVel;
  ros::Subscriber subSensor;
  float maxSpeed;
  float maxRotation;
  double safeDistance;
  ros::NodeHandle nh;
 public:
  Walker();
  Walker(const float& linear, const float& rotation,
         const double& safe);
  ~Walker();
  bool getObstacleDetected();
  void startMotion();
  void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void resetBot();
  float getMaxSpeed();
  void setMaxSpeed(const float& speed);
  float getMaxRotation();
  void setMaxRotation(const float& rotation);
  double getSafeDistance();
};
#endif  // INCLUDE_WALKER_HPP_
