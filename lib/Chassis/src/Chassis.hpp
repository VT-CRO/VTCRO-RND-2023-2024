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
#include "queue.h"

#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32MultiArray.h"

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
  Chassis();
  ~Chassis();

  void motorTest();
  void cmdVelTest();
  double* getWheelSpeeds();
  void meccanum_kinematics(geometry_msgs::Twist cmd_vel);

  ros::NodeHandle *_nh;

private:

  double _wheel_speeds[NUM_MOTORS];
  double _chassis_length, _chassis_width;

};

#endif // !CHASSIS_H
