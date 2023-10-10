/////////////////////////////////////////////////////////////
// Author: Domenic Marcelli   R&D Team VT CRO
// filename: MotorControl.c
// Last Modified: 10/07/2023
// Description:  This where the function defintions are kept
///////////////////////////////////////////////////////////// 

#include <MotorControl.h>
//Pin Macros Here
#define IN1 (PORTB |= (1<<6))   //output pin 14 of the teensy going to the IN1 pin of the Drive Carrier PORT B Pin 6
#define IN2 (PORTB |= (1<<7))    //output pin 15 of the teensy going to the IN2 pin of the Drive Carrier PORT B Pin 7  


void Motor_setPIDParams(){

}

void Motor_start(){

}

void Motor_dispatch(){

}

void Motor_pin_init(){
      //according to the I/O documentation this should change set bits 6 and 7 to 1 However... it is not
}

void Motor_setSpeed(){

}

void Motor_setDirection(){

}

void Motor_pidControlLoop(){
    
}