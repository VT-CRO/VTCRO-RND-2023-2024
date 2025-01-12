/////////////////////////////////////////////////////////////
// Author: Jayson De La Vega   R&D Team VT CRO
// filename: spin_task.h
// Last Modified: 1/24/2024
// Description:  This file contains the function definitions
//               for the spin task that periodically updates
//               (spins) the ros node handler.
//
//               Adapted from:
//               https://github.com/robosavvy/rosserial_stm32f1_tutorials/tree/master
///////////////////////////////////////////////////////////// 
#include "spin_task.h"
#include "arduino_freertos.h"

static ros::NodeHandle *nh_;

static void spinTask(void *pvParameters)
{
    TickType_t ui32WakeTime = xTaskGetTickCount();

    while (1)
    {
        taskENTER_CRITICAL();

        nh_->loginfo("Spinning...");
        nh_->spinOnce();
        
        taskEXIT_CRITICAL();

        xTaskDelayUntil(&ui32WakeTime, pdMS_TO_TICKS(ROS_SPIN_PERIOD));
    }
}

bool spinInitTask(ros::NodeHandle *nh)
{
    nh_ = nh;

    if (xTaskCreate(spinTask, "spin", 100, NULL, tskIDLE_PRIORITY + tskSPIN_PRIORITY, NULL) != pdTRUE)
    {
        nh_->logfatal("Failed to create spin task!");
        return 1;
    }
    return 0;
}