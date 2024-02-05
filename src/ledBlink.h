#ifndef LEDBLINK_HPP
#define LEDBLINK_HPP

#include "arduino_freertos.h"

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
    if (xTaskCreate(flashLED_task, "Blinking LED Task", 100, NULL, tskIDLE_PRIORITY + 1, NULL) != pdTRUE)
    {
        digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);
        return 1;
    }
    return 0;
}

#endif