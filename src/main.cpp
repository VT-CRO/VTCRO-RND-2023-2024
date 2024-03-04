#include "arduino_freertos.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>

#include "Chassis.hpp"
#include "MotorControl.h"

#include "HardwareDefs.h"

#include <Servo.h>

ros::NodeHandle nh;
Chassis chassis;

MotorControl m1(MOTOR_FL_IN1, MOTOR_FL_IN2);
MotorControl m2(MOTOR_FR_IN1, MOTOR_FR_IN2);
MotorControl m3(MOTOR_BR_IN1, MOTOR_BR_IN2);
MotorControl m4(MOTOR_BL_IN1, MOTOR_BL_IN2);

Servo thruster_arm_servo;
Servo thruster_rel_servo;

void sub_cb(const geometry_msgs::Twist& msg)
{
  chassis.meccanum_kinematics(msg);
}

void thruster_arm_pub_cb(const std_msgs::Int16& msg)
{
  thruster_arm_servo.write(msg.data);
}

void thrust_rel_pub_cb(const std_msgs::Int16& msg)
{
  thruster_rel_servo.write(msg.data);
}


ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &sub_cb);
ros::Subscriber<std_msgs::Int16> thruster_arm_sub("/thruster_angle", &thruster_arm_pub_cb);
ros::Subscriber<std_msgs::Int16> thruster_rel_sub("/thruster_release", &thrust_rel_pub_cb);


FLASHMEM __attribute__((noinline)) void setup()
{
  thruster_arm_servo.attach(2);
  thruster_rel_servo.attach(11);
  pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
  digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);

  nh.getHardware()->setBaud(115200);
  nh.initNode();

  nh.subscribe(sub);
  nh.subscribe(thruster_arm_sub);
  nh.subscribe(thruster_rel_sub);
}

void loop()
{
  double* speds = chassis.getWheelSpeeds();

  char buff[50];
  sprintf(buff, "%d %d %d %d", (int)speds[0], (int)speds[1], (int)speds[2], (int)speds[3]);
  nh.loginfo(buff);

  m1.Motor_start((int)speds[0]);
  m2.Motor_start((int)speds[1]);
  m3.Motor_start((int)speds[2]);
  m4.Motor_start((int)speds[3]);

  nh.spinOnce();
  
  delay(100);
}
