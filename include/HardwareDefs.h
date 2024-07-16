#ifndef HARDWAREDEFS_H
#define HARDWAREDEFS_H

#define MOTOR_FL_IN1 7
#define MOTOR_FL_IN2 6

#define MOTOR_FR_IN1 4
#define MOTOR_FR_IN2 5

#define MOTOR_BR_IN1 9
#define MOTOR_BR_IN2 8

#define MOTOR_BL_IN1 3
#define MOTOR_BL_IN2 2

#define QTR_PIN0 23
#define QTR_PIN1 15
#define QTR_PIN2 22
#define QTR_PIN3 14
#define QTR_PIN4 21
#define QTR_PIN5 26
#define QTR_PIN6 20
#define QTR_PIN7 27
#define QTR_PIN8 19
#define QTR_PIN9 41
#define QTR_PIN10 18
#define QTR_PIN11 40
#define QTR_PIN12 17
#define QTR_PIN13 39
#define QTR_PIN14 16
#define QTR_PIN15 38
#define QTR_THRESHOLD 900

#define START_LED_PIN 26
#define START_LED_THRESHOLD 650  // assuming a 2.7k Ohm resistance

#define QDC1_A 1
#define QDC1_B 2
#define QDC2_A 3
#define QDC2_B 4
#define QDC3_A 30
#define QDC4_B 31
#define QDC5_A 32
#define QDC6_B 33

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  1000 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2000 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define SMALL_BLOCK_MOTOR 11
#define BRIDGE_SERVO 12
#define BIG_BLOCK_LEFT -1   // looking directly at mech down pos = 0
#define BIG_BLOCK_RIGHT -1  // down pos = 180
#define BIG_BLOCK_RELEASE -1   //130 is retracted point

#define WHITE_PHOTO 24

#define LENGTH 9  // inches
#define WIDTH 10   // inches

#define WHEEL_RADIUS 1  // inches

#endif