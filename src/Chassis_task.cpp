#include <stdbool.h>
#include <stdint.h>
#include "Chassis_task.hpp"
#include "task.h"
#include "queue.h"
#include "geometry_msgs/Twist.h"

#define tskSUBSCRIBE_PRIORITY 2

static ros::NodeHandle *nh_;

QueueHandle_t g_pStrQueue;
QueueHandle_t g_pStrLenQueue;
const int STR_QUEUE_SIZE = 5;

// Callback for receiving string message from topic, sending it to the queue to be processed
static void str_cb(const geometry_msgs::Twist& msg)
{
  if (xQueueSendToBack(g_pStrQueue, &msg, 500) != pdPASS)
  {
    // Error. Queue is full
#ifdef USE_USBCON
    UARTprintf("String message queue full.\n");
#endif
  }
}

ros::Subscriber<geometry_msgs::Twist> str_sub("/cmd_vel", &str_cb);

// subscriber task - Processes data from the string message queue
static void subscribeTask(void *pvParameters)
{
  TickType_t ui32WakeTime;
  // Get the current tick count.
  ui32WakeTime = xTaskGetTickCount();

  geometry_msgs::Twist incoming_msg;
  uint32_t msg_x;
  while (1)
  {
    if (xQueueReceive(g_pStrQueue, &incoming_msg, 0) == pdPASS)
    {
      msg_x = incoming_msg.linear.x;
      xQueueSendToBack(g_pStrLenQueue, &msg_x, 500);
    }
    vTaskDelayUntil(&ui32WakeTime, 100);
  }
}

// Initialization of the subscriber task
// Also registers subscriber onto the node handle.
uint32_t subscribeInitTask(ros::NodeHandle *nh)
{
  nh_ = nh;
  nh_->subscribe(str_sub);

  g_pStrQueue = xQueueCreate(STR_QUEUE_SIZE, sizeof(geometry_msgs::Twist));
  g_pStrLenQueue = xQueueCreate(STR_QUEUE_SIZE, sizeof(uint32_t));

  // Init spin task
  if (xTaskCreate(subscribeTask, "subscribe", 100, NULL, tskIDLE_PRIORITY + tskSUBSCRIBE_PRIORITY, NULL) != pdTRUE)
  {
    return 1;
  }
  return 0;
}