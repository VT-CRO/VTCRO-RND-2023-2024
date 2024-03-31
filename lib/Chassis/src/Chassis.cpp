/////////////////////////////////////////////////////////////
// Author: Jayson De La Vega   R&D Team VT CRO
// modified by: Domenic Marcelli
// modification: changed PID timer task to pid_task on line 25
// filename: Chassis.cpp
// Last Modified: 1/24/2024
// Description:  This file contains class declarations for
//               a mecannum chassis object
/////////////////////////////////////////////////////////////
#include "Chassis.hpp"
#include <HardwareDefs.h>

void Chassis_init(Chassis_t* chassis_ptr, MotorControl m1, MotorControl m2, MotorControl m3, MotorControl m4) {
  // initialize motor tasks here
}

void Chassis_setVelocityCorrection(Chassis_t* chassis_ptr, uint16_t err) {
  chassis_ptr->velocityCorrection = err;
}

void Chassis_meccanum_kinematics(Chassis_t* chassis_ptr, geometry_msgs::Twist cmd_vel) {
  float x = cmd_vel.linear.x;
  float y = cmd_vel.linear.y;
  float w = cmd_vel.angular.z;

  float lw = LENGTH + WIDTH;

  // Standard inverse kinematic algorithm for a mecanum drive
  // adapted from:
  // https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html
  double speed_1 = x - y - (w * lw) - chassis_ptr->velocityCorrection;
  double speed_2 = x + y + (w * lw) + chassis_ptr->velocityCorrection;
  double speed_3 = x - y + (w * lw) - chassis_ptr->velocityCorrection;
  double speed_4 = x + y - (w * lw) + chassis_ptr->velocityCorrection;
}