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

Chassis::Chassis(){};

Chassis::~Chassis(){};

void Chassis::meccanum_kinematics(geometry_msgs::Twist cmd_vel) {
  float x = cmd_vel.linear.x;
  float y = cmd_vel.linear.y;
  float w = cmd_vel.angular.z;

  float lw = LENGTH + WIDTH;

  _wheel_speeds[0] = x - y - (w * lw);
  _wheel_speeds[1] = x + y + (w * lw);
  _wheel_speeds[2] = x - y + (w * lw);
  _wheel_speeds[3] = x + y - (w * lw);

  for (int i = 0; i < 4; ++i) {
    _wheel_speeds[i] = map(_wheel_speeds[i], -1, 1, -255, 255);
  }
}

double *Chassis::getWheelSpeeds() { return _wheel_speeds; }

// void Chassis::chassisControl_task(void *pvParameters)
// {
//     Chassis *instance = (Chassis *)pvParameters;

//     TickType_t ui32WakeTime = xTaskGetTickCount();

//     while (1)
//     {
//         instance->chassisControl();

//         xTaskDelayUntil(&ui32WakeTime, pdMS_TO_TICKS(CONTROL_LOOP_PERIOD));
//     }
// }

// void Chassis::chassisControl()
// {
//     // _cmd_vel.linear.y -= _line_following_gain * line_follower.getState();

//     // meccanum_kinematics(_cmd_vel);

//     // for (unsigned int i = 0; i < _motors.size(); ++i)
//     //     _motors[i].Motor_start(_wheel_speeds[i]);
// }

// void Chassis::motorTest()
// {
//     bool on = false;
//     while(1) {
//         if (on)
//             digitalWrite(arduino::LED_BUILTIN, arduino::HIGH);
//         else
//             digitalWrite(arduino::LED_BUILTIN, arduino::LOW);

//         on = !on;
//         Serial.println("Running motor Tests...");
//         for (unsigned int i = 0; i < _motors.size(); ++i) {
//             _motors[i].Motor_start(200);
//             delay(1000);
//         }
//     }
// }

// void Chassis::cmdVelTest()
// {
//     geometry_msgs::Twist up;    up.linear.x = 200;
//     geometry_msgs::Twist upLeft;    upLeft.linear.x = 200; upLeft.linear.y =
//     200; geometry_msgs::Twist left;  left.linear.y = 200;
//     geometry_msgs::Twist downLeft;  downLeft.linear.x = -200;
//     downLeft.linear.y = 200; geometry_msgs::Twist down;  down.linear.x =
//     -200; geometry_msgs::Twist downRight; downRight.linear.x = -200;
//     downRight.linear.y = -200; geometry_msgs::Twist right; right.linear.y =
//     -200; geometry_msgs::Twist upRight;   upRight.linear.y = -200;
//     upRight.linear.x = 200;

//     geometry_msgs::Twist cardinal_dirs[8] = {
//         up, upLeft, left, downLeft, down, downRight, right, upRight
//     };

//     bool on = false;

//     _line_following_gain = 0;

//     while (1) {
//         // for (int i = 0; i < 8; ++i)
//         // {
//         //     if (on)
//         //         digitalWrite(arduino::LED_BUILTIN, arduino::HIGH);
//         //     else
//         //         digitalWrite(arduino::LED_BUILTIN, arduino::LOW);

//         //     on = !on;
//         //     meccanum_kinematics(cardinal_dirs[i]);

//         //     for (unsigned int j = 0; j < _motors.size(); ++j)
//         //         _motors[j].Motor_start(_wheel_speeds[j]);
            
//         //     delay(500);
//         // }
//         meccanum_kinematics(cardinal_dirs[0]);
//         for (unsigned int j = 0; j < _motors.size(); ++j)
//             _motors[j].Motor_start(_wheel_speeds[j]);
//     }
// }
