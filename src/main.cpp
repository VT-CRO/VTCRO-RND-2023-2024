#include "arduino_freertos.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>

#include <spin_task.h>
#include <Chassis.h>

ros::NodeHandle nh;

FLASHMEM __attribute__((noinline))
void setup() {

  Chassis chassis;

  chassis.chassisTest();

  /*
  // Serial.begin(9600);

  nh.initNode();

  // Serial.println("Initializing spin task");
  nh.logdebug("Initializing spin task");

  delay(10000);

  if (spinInitTask(&nh))
  {
    // error2
    while(1);
  }

  // subscriber test

  Chassis chassis;

  if (chassis.initTask(&nh))
  {
    // error
    while(1);
  }
  */

  

  vTaskStartScheduler();

  while(1)
  {
    // Serial.println("Scheduler Failed! \n");
    nh.logfatal("Scheduler Failed!");
    Serial.flush();
    delay(1000);
  }
}

void loop() {
  delay(100);
}
