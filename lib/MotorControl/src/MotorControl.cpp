/////////////////////////////////////////////////////////////
// Author: Domenic Marcelli   R&D Team VT CRO
// filename: MotorControl.c
// Last Modified: 10/07/2023
// Description:  This where the function defintions are kept
///////////////////////////////////////////////////////////// 

//notes PWM example https://github.com/khoih-prog/Teensy_PWM  <- might be useful but all examples are in INO form
// might want to look at ECE 2564 notes for PWM Reference HW 11
//check platform IO path .platformio\packages\framework-arduinoteensy-ts\cores\teensy4 for pwm.c file
#include <MotorControl.h>

//check out his header c file for everything PWM related.

//Pin Macros Here

//might want to make a custom.
MotorControl::MotorControl(){
    outputPin1 = 14;
    outputPin2 = 15;
}

void MotorControl::Motor_setPIDParams(){

}

void MotorControl::Motor_start(){
    analogWrite(outputPin1, 50);
}

//don't do
void MotorControl::Motor_dispatch(){

}

//initalizes the pins 
void MotorControl::Motor_pin_init(){
      pinMode(outputPin1, 1);
      pinMode(outputPin2, 1);
}

void MotorControl::Motor_setSpeed(){

}

void MotorControl::Motor_setDirection(){

}

void MotorControl::Motor_pidControlLoop(){
    
}