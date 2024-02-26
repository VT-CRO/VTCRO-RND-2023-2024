#include "arduino_freertos.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Adafruit_TCS34725.h>

#include <spin_task.h>
#include <Chassis.h>
#include "QTR_8.h"
#include "QTR_8.cpp"

void subscriber_cb(const geometry_msgs::Twist& msg)
{
  Serial.printf("cmd_vel:\n\tlinear:\n\t\tx: %f\n\t\ty: %f\n\t\ty: %f\n\tangular:\t\tx: %f\n\t\ty: %f\n\t\ty: %f\n", msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z);
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &subscriber_cb);

FLASHMEM __attribute__((noinline))
void setup() {
  Serial.println("Starting! \n");
 /*
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
*/
  Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
  if (tcs.begin() == false) {
    while(1) {
      Serial.println("Unable to find color sensor!!");
    }
  }
  
  uint16_t clear, red, green, blue;
  while(1) {
    tcs.setInterrupt(false);
    delay(60);
    
    tcs.getRawData(&clear, &red, &green, &blue);
    Serial.print("\nclear: ");
    Serial.println(clear);
    Serial.print("red: ");
    Serial.println(red);
    Serial.print("green: ");
    Serial.println(green);
    Serial.print("blue: ");
    Serial.println(blue);

    delay(1000);
  }

  QTR_8 qtr(4);
  qtr.initTask();
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
