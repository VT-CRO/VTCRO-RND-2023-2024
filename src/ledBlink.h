#ifndef LEDBLINK_HPP
#define LEDBLINK_HPP

#include "arduino_freertos.h"

static TaskHandle_t *task_ptr;

void flashLED_task(void *parameters)
{

    bool toggle = 0;
    while (1)
    {
        if (toggle)
        {
            digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);
        }
        else
            digitalWriteFast(arduino::LED_BUILTIN, arduino::LOW);
        toggle = !toggle;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

bool flashLED_init()
{
    if (xTaskCreate(flashLED_task, "Blinking LED Task", 100, NULL, tskIDLE_PRIORITY + 1, task_ptr) != pdTRUE)
    {
        digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);
        return 1;
    }
    return 0;
}

void stopBlink()
{
    digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);
    vTaskSuspend(*task_ptr);
}

void startBlink()
{
    vTaskResume(*task_ptr);
}

#endif

void errorBlink()
{
    bool on = false;
    while (1) {
        if (on)
            digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);
        else 
            digitalWriteFast(arduino::LED_BUILTIN, arduino::LOW);
        on = !on;
    }
}