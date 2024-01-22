/////////////////////////////////////////////////////////////
// Author: Jayson De La Vega   R&D Team VT CRO
// filename: Chassis.h
// Last Modified: 1/24/2024
// Description:  This file contains class declarations for
//               a meccanum chassis object
///////////////////////////////////////////////////////////// 
#ifndef CHASSIS_H
#define CHASSIS_H

#include "MotorControl.h"
#include "Observer.hpp"
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "arduino_freertos.h"

#define NUM_MOTORS 4
#define CONTROL_LOOP_PERIOD 50

#define tskCHASSIS_PRIORITY 1

/*
 * This class specifies the class interface for the chassis.
 * The chassis, with the corresponding wheel numbers is as such:
 * 
 *                 ^  +x
 *                 |
 *          [1]---------[2]
 *           |   Front   |
 *   +y <--- |     x z+  |
 *           |           |
 *          [4]---------[3]
 */ 
class Chassis {
public:
  Chassis();
  Chassis(Chassis &&) = default;
  Chassis(const Chassis &) = default;
  Chassis &operator=(Chassis &&) = default;
  Chassis &operator=(const Chassis &) = default;
  ~Chassis();

  bool initTask(ros::NodeHandle *nh);

private:

  // Motor control
  MotorControl motors;
  int _wheel_speeds[NUM_MOTORS];

  // Line following
  Observer<int> line_follower;
  int _line_following_gain;

  double _chassis_length, _chassis_width;

  void subscriber_cb(const geometry_msgs::Twist &cmd_vel);
  ros::NodeHandle *_nh;
  ros::Subscriber<geometry_msgs::Twist, Chassis> sub;
  geometry_msgs::Twist _cmd_vel;

  void meccanum_kinematics(geometry_msgs::Twist cmd_vel);
  static void chassisControl_task(void * pvParameters);
  void chassisControl();
};

#endif // !CHASSIS_H
