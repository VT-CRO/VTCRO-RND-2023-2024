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
    : sub("/cmd_vel", &Chassis::subscriber_cb, this),
      // pub("/wheel_speeds", &wheels),
      enc1(1, 1, 2, 1, 2, 1, 0),
      enc2(2, 3, 4, 3, 4, 3, 0),
      enc3(3, 30, 31, 30, 31, 30, 0),
      enc4(4, 32, 33, 32, 33, 32, 0),
      m1(5, 6),
      m2(7, 8),
      m3(37, 36),
      m4(28, 29)
{
    _chassis_length = 1;
    _chassis_width = 1;

    m1.Motor_pin_init();
    m2.Motor_pin_init();
    m3.Motor_pin_init();
    m4.Motor_pin_init();

    // TODO: QRT_Sensor.attachObserver(line_follower)

    // enc1.init();
    // enc2.init();
    // enc3.init();
    // enc4.init();
}

bool Chassis::initTask(ros::NodeHandle *nh)
{
    _nh = nh;
    _nh->subscribe(sub);
    // _nh->advertise(pub);

    // initialize motor tasks
    // Serial.println("Chassis initializing task");
    _nh->loginfo("Chassis initializing task...");

    if (xTaskCreate(Chassis::chassisControl_task, "chassis control task", 2048, this, tskIDLE_PRIORITY + tskCHASSIS_PRIORITY, NULL) != pdTRUE)
    {
        _nh->logfatal("Failed to create Chassis task!");
        return 1;
    }
    _nh->loginfo("Successfully initialized Chassis task.");
    return 0;
}

void Chassis::subscriber_cb(const geometry_msgs::Twist &cmd_vel)
{
    _cmd_vel = cmd_vel;

    _nh->loginfo("Subscriber callback.");

    // Rosserial arrays are weird so you have to do this
    // wheels.data = _wheel_speeds;
    // wheels.data[0] = _wheel_speeds[0];
    // wheels.data[1] = _wheel_speeds[1];
    // wheels.data[2] = _wheel_speeds[2];
    // wheels.data[3] = _wheel_speeds[3];
    // wheels.data_length = 4;

    // pub.publish(&wheels);
}

void Chassis::meccanum_kinematics(geometry_msgs::Twist cmd_vel)
{
    float x = cmd_vel.linear.x;
    float y = cmd_vel.linear.y;
    float w = cmd_vel.angular.z;

    _wheel_speeds[0] = (-1 * (_chassis_length + _chassis_width) * w) + x - y;
    _wheel_speeds[1] = ((_chassis_length + _chassis_width) * w) + x + y;
    _wheel_speeds[2] = ((_chassis_length + _chassis_width) * w) + x - y;
    _wheel_speeds[3] = (-1 * (_chassis_length + _chassis_width) * w) + x + y;
}

// use as ros subscriber callback function
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
    _nh->loginfo("Chassis control loop");

    // to compensate for off-centeredness, we add chassis velocity in the y direction
    // _cmd_vel.linear.y -= _line_following_gain * line_follower.getState();

    _nh->loginfo("Calculated wheel speeds");
    meccanum_kinematics(_cmd_vel);

    // _nh->loginfo("Set wheel speeds");

    m1.Motor_start(_wheel_speeds[0]);
    m2.Motor_start(_wheel_speeds[1]);
    m3.Motor_start(_wheel_speeds[2]);
    m4.Motor_start(_wheel_speeds[3]);

    // Motor PID control loop handled in a separate RTOS thread
    // for (int i = 0; i < NUM_MOTORS; ++i) {

    // }
}

void Chassis::chassisTest()
{
    m1.Motor_start(100);

    while (1)
    {
        uint16_t enc1_pos = enc1.getPosition();
        uint16_t enc2_pos = enc2.getPosition();
        uint16_t enc3_pos = enc3.getPosition();
        uint16_t enc4_pos = enc4.getPosition();

        Serial.printf("Encoder Readings:\n\t-1: &d\n\t-2: &d\n\t-3: &d\n\t-4: &d\n", enc1_pos, enc2_pos, enc3_pos, enc4_pos);

        delay(5000);
    }
}