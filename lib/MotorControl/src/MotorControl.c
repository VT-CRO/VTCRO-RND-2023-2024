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

#define PIN14_CONFIG (DDRB |= (1<<6) )  //this might actually configure pin 14 as an output
#define PIN14_CONFIG (DDRB |= (1<<7) )  //this might actually configure pin 15 as an output 

void Motor_setPIDParams(){

}

void Motor_start(){
   // pwm_init();
    //don't know whether this is write. Refer to pwm.c
    //analogWriteFrequency(14, 1000.0);   // analogWriteFrequency write to 14, with a frequency 1 kHz
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