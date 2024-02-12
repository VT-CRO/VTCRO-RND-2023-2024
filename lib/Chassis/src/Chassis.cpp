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

Chassis::Chassis(std::vector<MotorControl> motors)
    : sub("/cmd_vel", &Chassis::subscriber_cb, this)
    // pub("/wheel_speeds", &wheels),

{
    _chassis_length = 11;
    _chassis_width = 11;

    _motors = motors;

    for (unsigned int i = 0; i < _motors.size(); ++i) {
        _motors[i].Motor_pin_init();
        // _motors[i].Motor_enablePIDTask();
    }
}

bool Chassis::initTask()
{
    // initialize motor tasks
    // Serial.println("Chassis initializing task");
    // _nh->loginfo("Chassis initializing task...");

    if (xTaskCreate(Chassis::chassisControl_task, "chassis control task", 2048, this, tskIDLE_PRIORITY + tskCHASSIS_PRIORITY, NULL) != pdTRUE)
    {
        // _nh->logfatal("Failed to create Chassis task!");
        return 1;
    }
    // _nh->loginfo("Successfully initialized Chassis task.");
    return 0;
}

void Chassis::initNode(ros::NodeHandle *nh)
{
    _nh = nh;
    _nh->subscribe(sub);
    // _nh->advertise(pub);
}

void Chassis::subscriber_cb(const geometry_msgs::Twist &cmd_vel)
{
    _cmd_vel = cmd_vel;

    _nh->loginfo("Subscriber callback.");
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

void Chassis::hahaRoutine()
{
    double inToS = 0.04;
    double degToS = 0.05;

    strafeLeft(inToS * 12);
    stop();
    forward(inToS * 12);
    stop();
    turnRight(degToS * 45);
    stop();
    forward(inToS * 6);
    stop();

    back(inToS * 6);
    turnLeft(degToS * 45);
    back(inToS * 14);

    strafeRight(inToS * 6);
    forward(inToS * 12);
    back(inToS * 13);

    strafeRight(inToS * 6);
    forward(inToS * 6);
    turnRight(degToS * 45);
    forward(inToS * 6);
    back(inToS * 6);
    turnLeft(degToS * 45);
    back(degToS * 9);

    strafeRight(inToS * 6);
    back(inToS * 6);

    stop();


    while(true) {
        stop();
    }
}

void Chassis::forward(float seconds)
{
    geometry_msgs::Twist up;    up.linear.x = 200;
    meccanum_kinematics(up);
    for (unsigned int j = 0; j < _motors.size(); ++j)
            _motors[j].Motor_start(_wheel_speeds[j]);
    delay(seconds * 1000);
}


void Chassis::back(float seconds)
{
    geometry_msgs::Twist back;    back.linear.x = -200;
    meccanum_kinematics(back);
    for (unsigned int j = 0; j < _motors.size(); ++j)
            _motors[j].Motor_start(_wheel_speeds[j]);
    delay(seconds * 1000);
}

void Chassis::strafeLeft(float seconds)
{
    geometry_msgs::Twist left;    left.linear.y = 200;
    meccanum_kinematics(left);
    for (unsigned int j = 0; j < _motors.size(); ++j)
            _motors[j].Motor_start(_wheel_speeds[j]);
    delay(seconds * 1000);
}

void Chassis::strafeRight(float seconds)
{
    geometry_msgs::Twist right;    right.linear.y = -200;
    meccanum_kinematics(right);
    for (unsigned int j = 0; j < _motors.size(); ++j)
            _motors[j].Motor_start(_wheel_speeds[j]);
    delay(seconds * 1000);
}