#include "Application.h"

#define tskAPP_FSM_PRIO 2

void Application_InitialState(Application_t* app_ptr, TickType_t*  startTime)
{
    TickType_t timeNow = xTaskGetTickCount();
    TickType_t dt = timeNow - *startTime;

    bool waitTimeExceeded = dt >= (5000 / portTICK_PERIOD_MS);
    bool autoStartDetected = analogRead(START_LED_PIN) >= START_LED_THRESHOLD;

    if (waitTimeExceeded || autoStartDetected)
    {
        *startTime = xTaskGetTickCount();
        app_ptr->state = BIG_BLOCK_PICKUP;
    }
}

void Application_BigBlockPickup(Application_t* app_ptr, TickType_t*  ui32WakeTime)
{
    // turn on small pickup motor

    vTaskDelayUntil(ui32WakeTime, 500 / portTICK_PERIOD_MS);

    // open big block release

    vTaskDelayUntil(ui32WakeTime, 500 / portTICK_PERIOD_MS);

    // put big block pickup down

    vTaskDelayUntil(ui32WakeTime, 500 / portTICK_PERIOD_MS);

    // move backwards

    vTaskDelayUntil(ui32WakeTime, 500 / portTICK_PERIOD_MS);

    // stop

    vTaskDelayUntil(ui32WakeTime, 500 / portTICK_PERIOD_MS);

    // close big block release servo

    vTaskDelayUntil(ui32WakeTime, 1000 / portTICK_PERIOD_MS);

    // move forward and put big block pickup up
    
    vTaskDelayUntil(ui32WakeTime, 500 / portTICK_PERIOD_MS);

    // stop an
    app_ptr->state = SMALL_BLOCK_PICKUP;
}

void Application_SmallBlockPickup(Application_t* app_ptr);
void Application_SmallBlockDropoff(Application_t* app_ptr);
void Application_ThrusterPickup(Application_t* app_ptr);
void Application_Bridge(Application_t* app_ptr);
void Application_ThrusterDropoff(Application_t* app_ptr);
void Application_PushButton(Application_t* app_ptr);
void Application_reset(Application_t* app_ptr);

void Application_fsm_task(void *pvParameters)
{
    TickType_t startTime;
    startTime = xTaskGetTickCount();
    Application_t* app_ptr = (Application_t *)pvParameters;

    while (1)
    {
        switch (app_ptr->state)
        {
            case INITIAL_STATE:
                Application_InitialState(app_ptr, &startTime);
                break;
            case BIG_BLOCK_PICKUP:
                Application_BigBlockPickup(app_ptr, &startTime);
                break;
            case SMALL_BLOCK_PICKUP:
                Application_SmallBlockPickup(app_ptr);
                break;
            case SMALL_BLOCK_DROPOFF:
                Application_SmallBlockDropoff(app_ptr);
                break;
            case THRUSTER_PICKUP:
                Application_ThrusterPickup(app_ptr);
                break;
            case BRIDGE:
                Application_Bridge(app_ptr);
                break;
            case THRUSTER_DROPOFF:
                Application_ThrusterDropoff(app_ptr);
                break;
            case PUSH_BUTTON:
                Application_PushButton(app_ptr);
                break;
            case END:
            default:
                Application_reset(app_ptr);
                break;
        }
    }
}

void Application_initHardware(Application_t* app_ptr)
{
    
}

bool Application_init_task(Application_t* app_ptr)
{
    /* === application initialization steps here === */

    app_ptr->state = INITIAL_STATE;

    /* ============================================== */

    BaseType_t ok;
    TaskHandle_t app_task_handle = NULL;

    ok = xTaskCreate(
        Application_fsm_task,
        "app_fsm",
        50,
        (void *) app_ptr,
        tskIDLE_PRIORITY + tskAPP_FSM_PRIO, 
        &app_task_handle);

    if (ok != pdPASS)
    {
        vTaskDelete(app_task_handle);
    }
    return (ok == pdPASS);
}

void Application_InitialState(Application_t* app_ptr)
{

}