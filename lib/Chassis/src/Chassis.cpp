/////////////////////////////////////////////////////////////
// Author: Jayson De La Vega   R&D Team VT CRO
// modified by: Domenic Marcelli
// modification: changed PID timer task to pid_task on line 25
// filename: Chassis.cpp
// Last Modified: 1/24/2024
// Description:  This file contains class declarations for
//               a mecannum chassis object
///////////////////////////////////////////////////////////// 
#include "Chassis.h"

Chassis::Chassis()
    : sub("/cmd_vel", &Chassis::subscriber_cb, this)
{
    for(int i = 0; i < NUM_MOTORS; ++i)
    {
        // TODO:
        //  - set motor in PID mode
        //  - set motor other parameters
        //  - set motor PID parameters
        //  - set motors to listen to encoders

        // initialize motor pid timers and set callback function
        
        //motors.pid_task(this);
    }

    // TODO: QRT_Sensor.attachObserver(line_follower)
}

bool Chassis::initTask(ros::NodeHandle *nh)
{
    _nh = nh;
    _nh->subscribe(sub);

    // initialize motor tasks

    if (xTaskCreate(Chassis::chassisControl_task, "chassis control task", 100, this, tskIDLE_PRIORITY + tskCHASSIS_PRIORITY, NULL) != pdTRUE)
        return 1;
    return 0;
}

void Chassis::subscriber_cb(const geometry_msgs::Twist &cmd_vel)
{
    _cmd_vel = cmd_vel;
}

void Chassis::meccanum_kinematics(geometry_msgs::Twist cmd_vel)
{
    float x = cmd_vel.linear.x;
    float y = cmd_vel.linear.y;
    float w = cmd_vel.angular.z;

    // Standard kinematic algorithms for meccanum chassis taken from:
    // https://nu-msr.github.io/navigation_site/lectures/derive_kinematics.html
    _wheel_speeds[0] = (-1 * (_chassis_length + _chassis_width) * w) + x - y;
    _wheel_speeds[1] = ((_chassis_length + _chassis_width) * w) + x + y;
    _wheel_speeds[2] = ((_chassis_length + _chassis_width) * w) + x - y;
    _wheel_speeds[3] = (-1 * (_chassis_length + _chassis_width) * w) + x + y;
}

// use as ros subscriber callback function
void Chassis::chassisControl_task(void * pvParameters)
{
    Chassis* instance = (Chassis *)pvParameters;

    TickType_t ui32WakeTime = xTaskGetTickCount();

    while (1) {
        instance->chassisControl();

        xTaskDelayUntil(&ui32WakeTime, pdMS_TO_TICKS(CONTROL_LOOP_PERIOD));
    }
}

void Chassis::chassisControl()
{
    // to compensate for off-centeredness, we add chassis velocity in the y direction
    _cmd_vel.linear.y -= _line_following_gain * line_follower.getState();

    meccanum_kinematics(_cmd_vel);

    // TODO: Change motors.Motor_setSpeed(_wheel_speeds[0], _wheel_speeds[1], _wheel_speeds[2], _wheel_speeds[3]);
    //assume encoders are going to have a function called setspeed;
    
    // Motor PID control loop handled in a separate RTOS thread
}