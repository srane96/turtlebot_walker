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
 * @file    walker.h
 * @author  Siddhesh Rane
 * @version 1.0
 * @brief walker class initiaation;
 *
 * @section DESCRIPTION
 *
 * C++ header file for Walker class which adds functionality of obstacle avoidance
 * to the gazebo turtlebot.
 */
#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
class Walker {
 private:
  /// Check if robot is in safe decision
  bool obstacleDetected;
  /// Publisher msg of type Twist
  geometry_msgs::Twist msg;
  /// Publisher object
  ros::Publisher pubVel;
  /// Subscriber object
  ros::Subscriber subSensor;
  /// MaxSpeed of turtlebot
  float maxSpeed;
  /// Max rotation angle
  float maxRotation;
  /// Allowable distance between bot and obstacle
  double safeDistance;
  /// Initiate NodeHandle object
  ros::NodeHandle nh;
 public:
  /**
   * @brief Default class constructor
   */
  Walker();
  /**
   * @brief Constructor with initial values
   * @param linear max linear speed of bot
   * @param roatation max rotation angle of bot
   * @param safe max allowable distance from obstacle
   * @return void
   */
  Walker(const float& linear, const float& rotation,
         const double& safe);
  /**
   * @brief Default destructor
   */
  ~Walker();
  /**
   * @brief Check if obstacle is present within safe distance
   * @return boolean
   */
  bool getObstacleDetected();
  /**
   * @brief Start running the bot
   * @return void
   */
  void startMotion();
  /**
   * @brief Callback function for subscriber
   * @param msg data from LaserScan node
   * @return void
   */
  void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  /**
   * @brief reset velocities of the bot
   * @return void
   */
  void resetBot();
  /**
   * @brief get maximum speed of the bot
   * @return float speed
   */
  float getMaxSpeed();
  /**
   * @brief set maximum speed of the bot
   * @param speed of the bot
   * @return void
   */
  void setMaxSpeed(const float& speed);
  /**
   * @brief get maximum rotation of the bot
   * @return float rotation angle
   */
  float getMaxRotation();
  /**
   * @brief set maximum rotation of the bot
   * @param rotation angle in radiance
   * @return void
   */
  void setMaxRotation(const float& rotation);
  /**
   * @brief get allowed distance from obstacle
   * @return double distance
   */
  double getSafeDistance();
};
#endif  // INCLUDE_WALKER_HPP_
