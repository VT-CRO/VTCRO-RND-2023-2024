/////////////////////////////////////////////////////////////
// Author: Domenic Marcelli  R&D Team VT CRO
// filename: MotorControl.h
// Last Modified: 10/27/2023
// Description:  This the header file for the motor controls.
///////////////////////////////////////////////////////////// 


//Notes: Here is a sensor I found that using the pins https://github.com/roggenkamps/teensy-thermoled/blob/master/tempsens.c
// we have four motors


#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include <avr/io.h>
#include <avr/pgmspace.h>
#include "arduino_freertos.h"

class MotorControl{

    public:

    MotorControl();

    //Sets the PID Params
    void Motor_setPIDParams();

    //Starts the motors
    void Motor_start();

    void Motor_dispatch();

    void Motor_pin_init();

    //sets the speed
    void Motor_setSpeed(int motor1, int motor2, int motor3, int motor4);

    //sets the motor direction
    void Motor_setDirection(int direction, int delay);

    //pid loop
    void Motor_pidControlLoop();

    void stopMove();

    private:

    void startMove();



    //don't know whether we will still need this
    struct _MotorStruct{
    int motorId ;
    int encoderPos;
    int targetSpeed;
    int maxSpeed; 
    } ;

    struct _Control{
        int forward_pin;
        int back_pin;
        int speed;
    };

    typedef _MotorStruct MotorStruct;
    typedef _Control Control;



    Control Motor1;
    Control Motor2;
    Control Motor3;
    Control Motor4;

    //These names are temporary.
    int motorOutputPWMPin1_1;
    int motorOutputPWMPin1_2;

    int motorOutputPWMPin2_1;
    int motorOutputPWMPin2_2;

    int motorOutputPWMPin3_1;
    int motorOutputPWMPin3_2;

    int motorOutputPWMPin4_1;
    int motorOutputPWMPin4_2;

    int speed;
};
//All the variables needed to control the motors


#endif