#include "arduino_freertos.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>

#include <spin_task.h>
#include <Chassis.h>
#include <ledBlink.h>

#include <HardwareDefs.h>

ros::NodeHandle nh;

FLASHMEM __attribute__((noinline)) void setup()
{
  std::vector<MotorControl> motors;
  motors.push_back(MotorControl(MOTOR_FL_IN1, MOTOR_FL_IN2)); // FL
  motors.push_back(MotorControl(MOTOR_FR_IN1, MOTOR_FR_IN2)); // FR
  motors.push_back(MotorControl(MOTOR_BR_IN1, MOTOR_BR_IN2)); // BR
  motors.push_back(MotorControl(MOTOR_BL_IN1, MOTOR_BL_IN2)); // BL

  // we'd also need to create encoders here and set Motors to listen to encoder count
  // QDC_Encoder enc1;
  // QDC_Encoder enc2;
  // QDC_Encoder enc3;
  // QDC_Encoder enc4;
  // enc1(1, 1, 2, 1, 2, 1, 0),
  // enc2(2, 3, 4, 3, 4, 3, 0),
  // enc3(3, 30, 31, 30, 31, 30, 0),
  // enc4(4, 32, 33, 32, 33, 32, 0),

  Chassis chassis(motors);

  pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
  digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);

  Serial.begin(9600);

  if (flashLED_init())
  {
    while (1)
      ;
  }

  chassis.hahaRoutine();
  // chassis.cmdVelTest();

  // nh.initNode();

  // Serial.println("Initializing spin task");
  // nh.logdebug("Initializing spin task");

  // delay(10000);

  // if (spinInitTask(&nh))
  // {
  //   // error2
  //   while (1)
  //     ;
  // }

  // if (chassis.initTask())
  // {
  //   // error
  //   while (1);
  // }

  // vTaskStartScheduler();

  while (1)
  {
    // Serial.println("Scheduler Failed! \n");
    nh.logfatal("Scheduler Failed!");
    Serial.flush();
    delay(1000);
  }
}

void loop()
{
}
