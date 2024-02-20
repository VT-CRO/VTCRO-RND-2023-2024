#include "arduino_freertos.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>

#include <spin_task.h>

#include "Chassis.hpp"
#include "MotorControl.h"

#include <HardwareDefs.h>

ros::NodeHandle nh;
Chassis chassis;

MotorControl m1(MOTOR_FL_IN1, MOTOR_FL_IN2);
MotorControl m2(MOTOR_FR_IN1, MOTOR_FR_IN2);
MotorControl m3(MOTOR_BR_IN1, MOTOR_BR_IN2);
MotorControl m4(MOTOR_BL_IN1, MOTOR_BL_IN2);

void sub_cb(const geometry_msgs::Twist& msg)
{
  chassis.meccanum_kinematics(msg);
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &sub_cb);

FLASHMEM __attribute__((noinline)) void setup()
{
  // std::vector<MotorControl> motors;
  // motors.push_back(); // FL
  // motors.push_back(); // FR
  // motors.push_back(); // BR
  // motors.push_back(); // BL

  pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
  digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  double* speds = chassis.getWheelSpeeds();

  char buff[50];
  sprintf(buff, "%d %d %d %d", (int)speds[0], (int)speds[1], (int)speds[2], (int)speds[3]);
  //sprintf(buff, "%d %d %d %d", m1.getSpeed(), m2.getSpeed(), m3.getSpeed(), m4.getSpeed());

  nh.loginfo(buff);
  m1.Motor_start((int)speds[0]);
  m2.Motor_start((int)speds[1]);
  m3.Motor_start((int)speds[2]);
  m4.Motor_start((int)speds[3]);

  nh.spinOnce();
  
  delay(500);
}
