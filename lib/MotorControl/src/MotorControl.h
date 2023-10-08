/////////////////////////////////////////////////////////////
// Author: Domenic Marcelli  R&D Team VT CRO
// filename: MotorControl.h
// Last Modified: 10/07/2023
// Description:  This the header file for the motor controls.
///////////////////////////////////////////////////////////// 


#ifdef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include <MotorControl.h>

//Pin Macros Here
#DEFINE IN1 14    //output pin 14 of the teensy going to the IN1 pin of the Drive Carrier 
#DEFINE IN2 15    //output pin 15 of the teensy going to the IN2 pin of the Drive Carrier

//All the variables needed to control the motors
struct _MotorStruct{
    int motorId ;
    int encoderPos;
    int targetSpeed;
    int maxSpeed; 
};

typedef _MotorStruct MotorStruct;

void Motor_setPIDParams();

void Motor_start();

void Motor_dispatch();

void Motor_pin_init();

void Motor_setSpeed();

void Motor_setDirection();

void Motor_pidControlLoop();

#endif