/////////////////////////////////////////////////////////////
// Author: Domenic Marcelli   R&D Team VT CRO
// filename: MotorControl.cpp
// Last Modified: 10/29/2023
// Description:  This where the function defintions for the 
//               Motor Controller are kept. Continously be updated
///////////////////////////////////////////////////////////// 

//notes PWM example https://github.com/khoih-prog/Teensy_PWM  <- might be useful but all examples are in INO form
// might want to look at ECE 2564 notes for PWM Reference HW 11
//check platform IO path .platformio\packages\framework-arduinoteensy-ts\cores\teensy4 for pwm.c file
#include <MotorControl.h>

//Motor Control Constructor
// Assigns values to the pin variables
// default PWM vals found at this link https://www.pjrc.com/teensy/td_pulse.html
// PinX_1 represents the PWM pin going to the IN1 on the Motor Controller
// PinX_2 represents the PWM pin going to the IN2 on the Motor Controller 
MotorControl::MotorControl(){
    motorOutputPWMPin1_1 = 1;  //4.482 kHz  - Pin 1 
    motorOutputPWMPin1_2 = 2; // 4.482 kHz - Pin 2


    motorOutputPWMPin2_1 = 3;  //4.482 kHz
    motorOutputPWMPin2_2 = 4;  // 4.482 kHz - Pin 4

    motorOutputPWMPin3_1 = 5;  //4.482 kHz - Pin 5
    motorOutputPWMPin3_2 = 6;  //4.482 kHz - Pin 6


    motorOutputPWMPin4_1 = 7;  //4.482 kHz - Pin 7
    motorOutputPWMPin4_2 = 8;  //4.482 kHz - Pin 8


}

void MotorControl::Motor_setPIDParams(){

}

// Motot_start send the PWM signals to the Motor Controller.
// Need to test if 0% duty cycle is the same as logic low
// default state moving forward. AnalogWrite values will be changed
void MotorControl::Motor_start(){
    analogWrite(motorOutputPWMPin1_1, 0);
    analogWrite(motorOutputPWMPin1_2, 0);

    analogWrite(motorOutputPWMPin2_1, 0);
    analogWrite(motorOutputPWMPin2_2, 0);

    analogWrite(motorOutputPWMPin3_1, 0);
    analogWrite(motorOutputPWMPin3_2, 0);

    analogWrite(motorOutputPWMPin4_1, 0);
    analogWrite(motorOutputPWMPin4_2, 0);
}

//don't do
void MotorControl::Motor_dispatch(){

}

//Motor_pin_init initalizates pins.
// honestly, we might want the constuctor to handle this
void MotorControl::Motor_pin_init(){
      pinMode(motorOutputPWMPin1_1, arduino::OUTPUT);
      pinMode(motorOutputPWMPin1_1, arduino::OUTPUT);

      pinMode(motorOutputPWMPin2_1, arduino::OUTPUT);
      pinMode(motorOutputPWMPin2_2, arduino::OUTPUT);

      pinMode(motorOutputPWMPin3_1, arduino::OUTPUT);
      pinMode(motorOutputPWMPin3_2, arduino::OUTPUT);

      pinMode(motorOutputPWMPin4_1, arduino::OUTPUT);
      pinMode(motorOutputPWMPin4_2, arduino::OUTPUT);
}

//Motor Speed sets the speed of the motors. 
//Values go from 0 - 255 for analogWrite. 
// I want to support negative values later
// to signify reverse.
void MotorControl::Motor_setSpeed(int motor1, int motor2, int motor3, int motor4){
    analogWrite(motorOutputPWMPin1_1, motor1);
    analogWrite(motorOutputPWMPin2_1, motor2);
    analogWrite(motorOutputPWMPin3_1, motor3);
    analogWrite(motorOutputPWMPin4_1, motor4);
}

//To change direction motors but be rotating in opposite directions.
//I used analogWrite so we can have more control over how fast we turn
// for now this might be unnessary. Wait for line following and PID loop stuff
void MotorControl::Motor_setDirection(int direction, int delay){

    //subject to change. We will want to control, angle of turn
    switch(direction){
        //forward
        case(0):
            //digitalWrite(motorOutIN1Pin1, arduino::HIGH);
            //digitalWrite(motorOutIN2Pin1, arduino::LOW);
            /*analogWrite(motorOutputPWMPin2_1, 0);
            analogWrite(motorOutputPWMPin3_1, 0);
            analogWrite(motorOutputPWMPin4_1, 0);*/
            break;
        //backward
        case(1):
            //analogWrite(motorOutputPWMPin1_1, 0);
            // analogWrite(motorOutputPWMPin2_1, 0);
            // analogWrite(motorOutputPWMPin3_1, 0);
            // analogWrite(motorOutputPWMPin4_1, 0);
            break;
        //don't worry about these for now.
        //counter clock wise
        case(2):
            //analogWrite(motorOutputPWMPin1_1, 0);
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