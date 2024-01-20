/////////////////////////////////////////////////////////////
// Author: Jayson De La Vega   R&D Team VT CRO
// filename: spin_task.h
// Last Modified: 1/24/2024
// Description:  This file contains the function declaration
//               for the spin task that periodically updates
//               (spins) the ros node handler.
//
//               Adapted from:
//               https://github.com/robosavvy/rosserial_stm32f1_tutorials/tree/master
///////////////////////////////////////////////////////////// 
#ifndef SPIN_TASK_H
#define SPIN_TASK_H

#include "arduino_freertos.h"
#include "ros.h"

#define ROS_SPIN_PERIOD 100
#define tskSPIN_PRIORITY 1

bool spinInitTask(ros::NodeHandle *nh);

#endif // !SPIN_TASK_H