/*
 * rosserial Publisher Example
 * Prints "hello world!"
 * here is an example of writing a ros node using the ardiuno ros serial lib
 * the main problem is that this is a .ino file so there might be some things that we can't
 * use because some things in ardiuno are c++. This is on the to do to investigate
 * I am also looking at alternatives using this website
 * https://micro.ros.org/docs/tutorials/core/teensy_with_arduino/
 */

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
