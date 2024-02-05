#include "arduino_freertos.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>

#include <spin_task.h>
#include <Chassis.h>
#include <ledBlink.h>
ros::NodeHandle nh;

FLASHMEM __attribute__((noinline)) void setup()
{
  pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
  digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);

  Serial.begin(9600);

  if (flashLED_init())
  {
    while (1)
    {
    };
  }
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

  // // subscriber test

  // Chassis chassis;

  // if (chassis.initTask(&nh))
  // {
  //   // error
  //   while (1)
  //     ;
  // }

  vTaskStartScheduler();

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
  delay(100);
}
