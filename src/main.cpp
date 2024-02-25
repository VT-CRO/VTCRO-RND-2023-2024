#include "Arduino_FreeRTOS.h"
#include "Arduino.h"

#include <ros.h>
#include <geometry_msgs/Twist.h>

#include <spin_task.h>

#include "Chassis.hpp"
#include "MotorControl.h"

#include "HardwareDefs.h"

// #include <Servo.h>

// ros::NodeHandle nh;
// Chassis chassis;

// MotorControl m1(MOTOR_FL_IN1, MOTOR_FL_IN2);
// MotorControl m2(MOTOR_FR_IN1, MOTOR_FR_IN2);
// MotorControl m3(MOTOR_BR_IN1, MOTOR_BR_IN2);
// MotorControl m4(MOTOR_BL_IN1, MOTOR_BL_IN2);

// void sub_cb(const geometry_msgs::Twist &msg)
// {
//   chassis.meccanum_kinematics(msg);
// }

// ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &sub_cb);

// // Servo servo;

// // FLASHMEM __attribute__((noinline))
// void setup()
// {
//   // servo.attach(6);
//   // std::vector<MotorControl> motors;
//   // motors.push_back(m1); // FL
//   // motors.push_back(); // FR
//   // motors.push_back(); // BR
//   // motors.push_back(); // BL

//   // the ardiuno commands don't work now with the version of the freeRTOS lib I switched to
//   pinMode(LED_BUILTIN, OUTPUT);
//   digitalWrite(LED_BUILTIN, HIGH);

//   nh.getHardware()->setBaud(115200);
//   nh.initNode();
//   nh.subscribe(sub);
// }

// void loop()
// {

//   // Serial.println("Hey I got to here");
//   double *speds = chassis.getWheelSpeeds();

//   char buff[50];
//   sprintf(buff, "%d %d %d %d", (int)speds[0], (int)speds[1], (int)speds[2], (int)speds[3]);
//   nh.loginfo(buff);
//   // sprintf(buff, "%d %d %d %d", m1.getSpeed(), m2.getSpeed(), m3.getSpeed(), m4.getSpeed());

//   m1.Motor_start((int)speds[0]);
//   // m1.logState(nh);
//   m2.Motor_start((int)speds[1]);
//   m3.Motor_start((int)speds[2]);
//   m4.Motor_start((int)speds[3]);

//   nh.spinOnce();

//   delay(100);

//   // servo.write(110); // 75 for full forward, 120 for reverse
//   // delay(3000);
//   // servo.write(0);
//   // delay(1000);
//   // servo.write(75);
//   // delay(3000);
//   // servo.write(0);
//   // delay(5000);
//}
