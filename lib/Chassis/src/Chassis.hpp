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

#include "MotorControl.h"

#define NUM_MOTORS 4
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
typedef struct {
  MotorControl* m1;
  MotorControl* m2;
  MotorControl* m3;
  MotorControl* m4;

  uint16_t velocityCorrection;
  double velocityCorrectionGain;
} Chassis_t;

void Chassis_init(Chassis_t* chassis_ptr);
void Chassis_setVelocityCorrection(Chassis_t* chassis_ptr, uint16_t err);
void Chassis_meccanum_kinematics(Chassis_t* chassis_ptr, geometry_msgs::Twist cmd_vel);

#endif // !CHASSIS_H
