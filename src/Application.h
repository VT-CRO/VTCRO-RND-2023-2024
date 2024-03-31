#ifndef APPLICATION_H
#define APPLICATION_H

#include "arduino_freertos.h"
#include "HardwareDefs.h"

/*
 * This type defines the different states of the small block system
 *
 * INTAKE: Spin the motors towards the robot to intake blocks
 * OUTTAKE: Spin the motors away from the robot to release blocks
 * OFF: Turn motors off
 *
 */
typedef enum {
    INTAKE,
    OUTTAKE,
    OFF
} small_pickup_state_t;

typedef enum {
    INITIAL_STATE,
    BIG_BLOCK_PICKUP,
    SMALL_BLOCK_PICKUP,
    SMALL_BLOCK_DROPOFF,
    THRUSTER_PICKUP,
    BRIDGE,
    THRUSTER_DROPOFF,
    PUSH_BUTTON,
    END
} application_state_t;

typedef struct {
    application_state_t state;
} Application_t;

bool Application_init_task(Application_t *app_ptr);



#endif