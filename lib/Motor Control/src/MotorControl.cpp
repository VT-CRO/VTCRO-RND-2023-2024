/////////////////////////////////////////////////////////////
// Author: Domenic Marcelli   R&D Team VT CRO
// filename: MotorControl.cpp
// Last Modified: 10/27/2023
// Description:  This where the function defintions are kept
///////////////////////////////////////////////////////////// 

//notes PWM example https://github.com/khoih-prog/Teensy_PWM  <- might be useful but all examples are in INO form
// might want to look at ECE 2564 notes for PWM Reference HW 11
//check platform IO path .platformio\packages\framework-arduinoteensy-ts\cores\teensy4 for pwm.c file
#include <MotorControl.h>

//check out his header c file for everything PWM related.

//Pin Macros Here

//might want to make a custom? Might just want to keep this hard coded.
//https://www.pjrc.com/teensy/td_pulse.html <- default PWM values for pins
//if we want more control then we need to find a way to change the timer
//related to these pins

//D pins control speed and 
//Need to see if I can do this with only 4 pins. Check Seniors code 
MotorControl::MotorControl(){
    motorOutputPWMPin1 = 37;  //4.482 kHz //37 - 
    int motorOutIN1Pin1 = 19;      //IN1 Digital Pin
    int motorOutIN2Pin1 = 18;      //IN2 Digital Pin

    motorOutputPWMPin2_1 = 36;  //4.482 kHz
    motorOutputPWMPin3_1 = 28;  //4.482 kHz
    motorOutputPWMPin4_1 = 29;  //4.482 kHz

}

void MotorControl::Motor_setPIDParams(){

}

//Starts up the motors by sending an analog pwm wave.
void MotorControl::Motor_start(){
    analogWrite(motorOutputPWMPin1, 0);
    digitalWrite(motorOutIN1Pin1, arduino::LOW);
    digitalWrite(motorOutIN2Pin1, arduino::LOW);


    analogWrite(motorOutputPWMPin2_1, 0);
    analogWrite(motorOutputPWMPin3_1, 0);
    analogWrite(motorOutputPWMPin4_1, 0);
}

//don't do
void MotorControl::Motor_dispatch(){

}

//initalizes the pins 
void MotorControl::Motor_pin_init(){
      pinMode(motorOutputPWMPin1, arduino::OUTPUT);
      pinMode(motorOutIN1Pin1, arduino::OUTPUT);
      pinMode(motorOutIN2Pin1, arduino::OUTPUT);

      pinMode(motorOutputPWMPin2_1, arduino::OUTPUT);
    //   pinMode(motorOutputPWMPin3_1, arduino::OUTPUT);
    //   pinMode(motorOutputPWMPin4_1, arduino::OUTPUT);
}

//Values go from 0 - 255 for analogWrite. Needed to be find some convention
//to make value. 
void MotorControl::Motor_setSpeed(int value){
    analogWrite(motorOutputPWMPin1, value);
    analogWrite(motorOutputPWMPin2_1, value);
    analogWrite(motorOutputPWMPin3_1, value);
    analogWrite(motorOutputPWMPin4_1, value);
}

//To change direction motors but be rotating in opposite directions.
//I used analogWrite so we can have more control over how fast we turn
void MotorControl::Motor_setDirection(int direction, int delay){

    //subject to change. We will want to control, angle of turn
    switch(direction){
        //forward
        case(0):
            digitalWrite(motorOutIN1Pin1, arduino::HIGH);
            digitalWrite(motorOutIN2Pin1, arduino::LOW);
            /*analogWrite(motorOutputPWMPin2_1, 0);
            analogWrite(motorOutputPWMPin3_1, 0);
            analogWrite(motorOutputPWMPin4_1, 0);*/
            break;
        //backward
        case(1):
            //analogWrite(motorOutputPWMPin1, 0);
            // analogWrite(motorOutputPWMPin2_1, 0);
            // analogWrite(motorOutputPWMPin3_1, 0);
            // analogWrite(motorOutputPWMPin4_1, 0);
            break;
        //counter clock wise
        case(2):
            //analogWrite(motorOutputPWMPin1, 0);
            // analogWrite(motorOutputPWMPin2_1, 0);
            // analogWrite(motorOutputPWMPin3_1, 0);
            // analogWrite(motorOutputPWMPin4_1, 0);
            break;
        //clock wise
        case(3):
            //analogWrite(motorOutputPin1, 0);
            // analogWrite(motorOutputPWMPin2_1, 0);
            // analogWrite(motorOutputPWMPin3_1, 0);
            // analogWrite(motorOutputPWMPin4_1, 0);
            break;
    }
}

void MotorControl::Motor_pidControlLoop(){
    
}