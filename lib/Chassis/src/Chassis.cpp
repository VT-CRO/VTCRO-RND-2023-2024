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
#include "HardwareDefs.h"

Chassis::Chassis(std::vector<MotorControl> motors)
    : sub("/cmd_vel", &Chassis::subscriber_cb, this)

{
    _motors = motors;

    for (unsigned int i = 0; i < _motors.size(); ++i) {
        _motors[i].Motor_pin_init();
    }
}

bool Chassis::initTask()
{
    if (xTaskCreate(Chassis::chassisControl_task, "chassis control task", 2048, this, tskIDLE_PRIORITY + tskCHASSIS_PRIORITY, NULL) != pdTRUE)
    {
        return 1;
    }
    return 0;
}

void Chassis::initNode(ros::NodeHandle *nh)
{
    _nh = nh;
    _nh->subscribe(sub);
}

void Chassis::subscriber_cb(const geometry_msgs::Twist &cmd_vel)
{
    taskENTER_CRITICAL();

    _nh->loginfo("Subscriber callback.");

    // might have to push data to queue for thread-safe data receive

    taskEXIT_CRITICAL();
}

void Chassis::meccanum_kinematics(geometry_msgs::Twist cmd_vel)
{
    float x = cmd_vel.linear.x;
    float y = cmd_vel.linear.y;
    float w = cmd_vel.angular.z;

    float lw = LENGTH + WIDTH;

    _wheel_speeds[0] = x + y - (lw * w / WHEEL_RADIUS);
    _wheel_speeds[1] = x + y + (lw * w / WHEEL_RADIUS);
    _wheel_speeds[2] = x - y + (lw * w / WHEEL_RADIUS);
    _wheel_speeds[3] = x + y - (lw * w / WHEEL_RADIUS);
}

void Chassis::chassisControl_task(void *pvParameters)
{
    Chassis *instance = (Chassis *)pvParameters;

    TickType_t ui32WakeTime = xTaskGetTickCount();

    while (1)
    {
        instance->chassisControl();

        xTaskDelayUntil(&ui32WakeTime, pdMS_TO_TICKS(CONTROL_LOOP_PERIOD));
    }
}

void Chassis::chassisControl()
{
    // to compensate for off-centeredness, we add chassis velocity in the y direction
    _cmd_vel.linear.y -= _line_following_gain * line_follower.getState();

    meccanum_kinematics(_cmd_vel);

    // Motor PID control loop handled in a separate RTOS thread
    for (unsigned int i = 0; i < _motors.size(); ++i)
        _motors[i].Motor_start(_wheel_speeds[i]);
}

void Chassis::motorTest()
{
    bool on = false;
    while(1) {
        if (on)
            digitalWrite(arduino::LED_BUILTIN, arduino::HIGH);
        else    
            digitalWrite(arduino::LED_BUILTIN, arduino::LOW);

        on = !on;
        Serial.println("Running motor Tests...");
        for (unsigned int i = 0; i < _motors.size(); ++i) {
            _motors[i].Motor_start(200);
            delay(1000);
        }
    }
}

void Chassis::cmdVelTest()
{
    geometry_msgs::Twist up;    up.linear.x = 200;
    geometry_msgs::Twist upLeft;    upLeft.linear.x = 200; upLeft.linear.y = 200;
    geometry_msgs::Twist left;  left.linear.y = 200;
    geometry_msgs::Twist downLeft;  downLeft.linear.x = -200; downLeft.linear.y = 200;
    geometry_msgs::Twist down;  down.linear.x = -200;
    geometry_msgs::Twist downRight; downRight.linear.x = -200; downRight.linear.y = -200;
    geometry_msgs::Twist right; right.linear.y = -200;
    geometry_msgs::Twist upRight;   upRight.linear.y = -200; upRight.linear.x = 200;

    geometry_msgs::Twist cardinal_dirs[8] = {
        up, upLeft, left, downLeft, down, downRight, right, upRight
    };

    bool on = false;

    _line_following_gain = 0;

    while (1) {
        // for (int i = 0; i < 8; ++i)
        // {
        //     if (on)
        //         digitalWrite(arduino::LED_BUILTIN, arduino::HIGH);
        //     else    
        //         digitalWrite(arduino::LED_BUILTIN, arduino::LOW);

        //     on = !on;
        //     meccanum_kinematics(cardinal_dirs[i]);

        //     for (unsigned int j = 0; j < _motors.size(); ++j)
        //         _motors[j].Motor_start(_wheel_speeds[j]);
            
        //     delay(500);
        // }
        meccanum_kinematics(cardinal_dirs[0]);
        for (unsigned int j = 0; j < _motors.size(); ++j)
            _motors[j].Motor_start(_wheel_speeds[j]);
    }
}