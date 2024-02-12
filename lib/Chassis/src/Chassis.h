/////////////////////////////////////////////////////////////
// Author: Jayson De La Vega   R&D Team VT CRO
// filename: Chassis.h
// Last Modified: 1/24/2024
// Description:  This file contains class declarations for
//               a meccanum chassis object
///////////////////////////////////////////////////////////// 
#ifndef CHASSIS_H
#define CHASSIS_H

#include "arduino_freertos.h"

#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32MultiArray.h"

#include "MotorControl.h"
#include "Observer.hpp"
#include "QDC_Encoder.h"

#define NUM_MOTORS 4
#define CONTROL_LOOP_PERIOD 500

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
  Chassis(std::vector<MotorControl> motors);
  Chassis(Chassis &&) = default;
  Chassis(const Chassis &) = default;
  Chassis &operator=(Chassis &&) = default;
  Chassis &operator=(const Chassis &) = default;
  ~Chassis();

  bool initTask();
  void initNode(ros::NodeHandle *nh);
  void motorTest();
  void cmdVelTest();
  void hahaRoutine();

  void forward(float);
  void back(float);
  void strafeLeft(float);
  void strafeRight(float);
  void turnLeft(float);
  void turnRight(float);
  void stop();

  ros::NodeHandle *_nh;

private:

  std::vector<MotorControl> _motors;
  int32_t _wheel_speeds[NUM_MOTORS];

  // Line following
  Observer<int> line_follower;
  int _line_following_gain = 0;

  double _chassis_length, _chassis_width;

  void subscriber_cb(const geometry_msgs::Twist &cmd_vel);
  ros::Subscriber<geometry_msgs::Twist, Chassis> sub;
  // ros::Publisher pub;
  geometry_msgs::Twist _cmd_vel;
  std_msgs::Int32MultiArray wheels;

  void meccanum_kinematics(geometry_msgs::Twist cmd_vel);
  static void chassisControl_task(void * pvParameters);
  void chassisControl();
};

#endif // !CHASSIS_H
