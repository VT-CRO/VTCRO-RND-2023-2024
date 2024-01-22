#include "arduino_freertos.h"
#include <micro_ros_platformio.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

#include <spin_task.h>
#include <Chassis.h>

void subscriber_cb(const geometry_msgs::Twist& msg)
{
  Serial.printf("cmd_vel:\n\tlinear:\n\t\tx: %f\n\t\ty: %f\n\t\ty: %f\n\tangular:\t\tx: %f\n\t\ty: %f\n\t\ty: %f\n", msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z);
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &subscriber_cb);

FLASHMEM __attribute__((noinline))
void setup() {

  nh.initNode();
  nh.subscribe(sub);

  if (spinInitTask(&nh))
  {
    // error
    while(1);
  }

  // subscriber test

  // Chassis chassis;

  // if (chassis.initTask(&nh))
  // {
  //   // error
  //   while(1);
  // }

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
