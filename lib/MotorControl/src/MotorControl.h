#ifdef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include <MotorControl.h>

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