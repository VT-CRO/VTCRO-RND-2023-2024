/////////////////////////////////////////////////////////////
// Author: Domenic Marcelli   R&D Team VT CRO
// filename: MotorControl.c
// Last Modified: 10/07/2023
// Description:  This where the function defintions are kept
///////////////////////////////////////////////////////////// 

//notes PWM example https://github.com/khoih-prog/Teensy_PWM  <- might be useful but all examples are in INO form
// might want to look at ECE 2564 notes for PWM Reference HW 11
//check platform IO path .platformio\packages\framework-arduinoteensy-ts\teensy4 for pwm.c file
#include <MotorControl.h>
//Pin Macros Here
//#define IN1 (PORTB |= (1<<6))   //output pin 14 of the teensy going to the IN1 pin of the Drive Carrier PORT B Pin 6  Maybe...
//#define IN2 (PORTB |= (1<<7))    //output pin 15 of the teensy going to the IN2 pin of the Drive Carrier PORT B Pin 7  Maybe...

#define PIN14_CONFIG (DDRB |= (1<<6) )  //this might actually configure pin 14 as an output
#define PIN14_CONFIG (DDRB |= (1<<7) )  //this might actually configure pin 15 as an output 

void Motor_setPIDParams(){

}

void Motor_start(){

}

void Motor_dispatch(){

}

void Motor_pin_init(){
      
}

void Motor_setSpeed(){

}

void Motor_setDirection(){

}

void Motor_pidControlLoop(){
    
}