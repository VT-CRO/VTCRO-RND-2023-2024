/////////////////////////////////////////////////////////////
// Author: Domenic Marcelli  R&D Team VT CRO
// filename: MotorControl.h
// Last Modified: 10/07/2023
// Description:  This the header file for the motor controls.
///////////////////////////////////////////////////////////// 


//Notes: Here is a sensor I found that using the pins https://github.com/roggenkamps/teensy-thermoled/blob/master/tempsens.c


#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include <avr/io.h>
#include <avr/pgmspace.h>
//#include <pwm.c>

//All the variables needed to control the motors
typedef struct{
    int motorId ;
    int encoderPos;
    int targetSpeed;
    int maxSpeed; 
    
} _MotorStruct;

typedef _MotorStruct MotorStruct;

void Motor_setPIDParams();

void Motor_start();

void Motor_dispatch();

void Motor_pin_init();

void Motor_setSpeed();

void Motor_setDirection();

void Motor_pidControlLoop();

#endif