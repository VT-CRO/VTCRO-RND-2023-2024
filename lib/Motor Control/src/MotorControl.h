/////////////////////////////////////////////////////////////
// Author: Domenic Marcelli  R&D Team VT CRO
// filename: MotorControl.h
// Last Modified: 10/27/2023
// Description:  This the header file for the motor controls.
///////////////////////////////////////////////////////////// 


//Notes: Here is a sensor I found that using the pins https://github.com/roggenkamps/teensy-thermoled/blob/master/tempsens.c
// we have four motors

//After discussion with Jason, make this general and one motor at a time
//so get rid of multiple pins and make it only one

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include <avr/io.h>
#include <avr/pgmspace.h>
#include "arduino_freertos.h"

#define tskPID_PRIORITY 7  //can be changed. Up for discussion

#define PID_LOOP_PERIOD 50

class MotorControl{

    public:

    MotorControl(int pin1, int pin2);

    //Sets the PID Params
    void Motor_setPIDParams(float P, float I, float D);

    // Enable speed PID control
    void Motor_enablePIDTask();

    //Starts the motors and changes speed
    void Motor_start(int newSpeed);

    void Motor_pin_init();

    //sets the motor direction
    //Might not need this
    void Motor_setDirection(int direction, int delay);

    //task for freeRTOS
    static void pid_task(void * pidParams);

    //pid loop
    void Motor_pidControlLoop();

    void Motor_stopMove();

    int getSpeed();

    private:

    //changes the go and no_go pins depending on whether the speed is positive or negative 
    void checkDirection(int speed);

    bool pidMode;

    int speed;

    int current_velocity;
    int goal_velocity;

    int last_error;

    int go_pin;
    int no_go_pin;

    struct _PinAssign {
        int in1;
        int in2;
    };

    typedef _PinAssign PinAssign;

    PinAssign Assignments;
 
    int motorP;
    int motorI;
    int motorD;


};
//All the variables needed to control the motors


#endif