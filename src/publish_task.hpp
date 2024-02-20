#ifndef PUBLISH_TASK_H
#define PUBLISH_TASK_H

#include "arduino_freertos.h"
#include "ros.h"

uint32_t publishInitTask(ros::NodeHandle *nh);

#endif // PUBLISH_TASK_H