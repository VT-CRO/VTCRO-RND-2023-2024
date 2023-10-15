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

//might want to make a custom? Might just want to keep this hard coded.
MotorControl::MotorControl(){
    motorOutputPin1 = 14;
    motorOutputPin2 = 15;
    motorOutputPin3 = 18;
    motorOutputPin4 = 19;
}

void MotorControl::Motor_setPIDParams(){

}

//Starts up the motors by sending an analog pwm wave.
void MotorControl::Motor_start(){
    analogWrite(motorOutputPin1, 0);
}

//don't do
void MotorControl::Motor_dispatch(){

}

//initalizes the pins 
void MotorControl::Motor_pin_init(){
      pinMode(motorOutputPin1, arduino::OUTPUT);
      pinMode(motorOutputPin2, arduino::OUTPUT);
      pinMode(motorOutputPin3, arduino::OUTPUT);
      pinMode(motorOutputPin4, arduino::OUTPUT);
}

//Values go from 0 - 255 for analogWrite. Needed to be find some convention
//to make value. 
void MotorControl::Motor_setSpeed(int value){
    analogWrite(motorOutputPin1, value);
    analogWrite(motorOutputPin2, value);
}

//To change direction motors but be rotating in opposite directions.
void MotorControl::Motor_setDirection(int direction, int delay){

    //subject to change. We will want to control, angle of turn
    switch(direction){
        //forward
        case(0):
            analogWrite(motorOutputPin1, 0);
            analogWrite(motorOutputPin2, 0);
            analogWrite(motorOutputPin3, 0);
            analogWrite(motorOutputPin4, 0);
            break;
        //backward
        case(1):
            analogWrite(motorOutputPin1, 0);
            analogWrite(motorOutputPin2, 0);
            analogWrite(motorOutputPin3, 0);
            analogWrite(motorOutputPin4, 0);
            break;
        //counter clock wise
        case(2):
            analogWrite(motorOutputPin1, 0);
            analogWrite(motorOutputPin2, 0);
            analogWrite(motorOutputPin3, 0);
            analogWrite(motorOutputPin4, 0);
            break;
        //clock wise
        case(3):
            analogWrite(motorOutputPin1, 0);
            analogWrite(motorOutputPin2, 0);
            analogWrite(motorOutputPin3, 0);
            analogWrite(motorOutputPin4, 0);
            break;
    }
}

void MotorControl::Motor_pidControlLoop(){
    
}