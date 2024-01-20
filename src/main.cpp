#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <ros.h>

#include <spin_task.h>
#include <Chassis.h>

ros::NodeHandle nh;

FLASHMEM __attribute__((noinline))
void setup() {

  nh.initNode();

  if (spinInitTask(&nh))
  {
    // error
    while(1);
  }

  Chassis chassis;

  if (chassis.initTask(&nh))
  {
    // error
    while(1);
  }

  vTaskStartScheduler();

  while(1)
  {
    Serial.println("Scheduler Failed! \n");
    Serial.flush();
    delay(1000);
  }
}

void loop() {
  delay(100);
}
