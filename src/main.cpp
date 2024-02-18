#include "arduino_freertos.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>

#include <spin_task.h>
#include <Chassis.h>
#include <MotorControl.h>
#include <ledBlink.h>

#include "HardwareDefs.h"

ros::NodeHandle nh;

FLASHMEM __attribute__((noinline)) void setup()
{ 
  std::vector<MotorControl> motors;
  motors.push_back(MotorControl(MOTOR_FL_IN1, MOTOR_FL_IN2)); // FL
  motors.push_back(MotorControl(MOTOR_FR_IN1, MOTOR_FR_IN2)); // FR
  motors.push_back(MotorControl(MOTOR_BR_IN1, MOTOR_BR_IN2)); // BR
  motors.push_back(MotorControl(MOTOR_BL_IN1, MOTOR_BL_IN2)); // BL
  
  Chassis chassis(motors);

  // nh.initNode();

  // pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
  // digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);

  // if (spinInitTask(&nh))
  // {
  //   errorBlink();
  // }

  // if (chassis.initTask())
  // {
  //   errorBlink();
  // }

  // chassis.initNode(&nh);

  // vTaskStartScheduler();

  // while (1)
  // {
  //   errorBlink();
  // }
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
