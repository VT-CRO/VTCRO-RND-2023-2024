#include <stdbool.h>
#include <stdint.h>
#include "publish_task.hpp"
#include "task.h"
#include "queue.h"
#include "std_msgs/Int32.h"

#define tskPUBLISH_PRIORITY 2

static ros::NodeHandle *nh_;
std_msgs::Int32 int_msg;
ros::Publisher count_pub("char_count", &int_msg);

extern QueueHandle_t g_pStrLenQueue;

// Publisher task - Receives data from the queue and places it on a Int message.
static void publishTask(void *pvParameters)
{
  TickType_t ui32WakeTime;
  // Get the current tick count.
  ui32WakeTime = xTaskGetTickCount();

  while (1)
  {
    // Publish message to be transmitted.
    if (nh_->connected())
    {
      if(xQueueReceive(g_pStrLenQueue, &int_msg.data, 0) == pdPASS)
      {
        count_pub.publish(&int_msg);
      }
    }

    vTaskDelayUntil(&ui32WakeTime, 50);
  }
}

// Initializations for the publish task.
// Registers publisher on node handler.
uint32_t publishInitTask(ros::NodeHandle *nh)
{
  nh_ = nh;
  nh_->advertise(count_pub);

  // Init spin task
  if (xTaskCreate(publishTask, "publish", 50, NULL, tskIDLE_PRIORITY + tskPUBLISH_PRIORITY, NULL) != pdTRUE)
  {
    return 1;
  }
  return 0;
}