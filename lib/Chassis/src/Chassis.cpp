#include "Chassis.h"

Chassis* Chassis::instance;

Chassis::Chassis()
{
    instance = this;

    for(int i = 0; i < NUM_MOTORS; ++i)
    {
        // set motor in PID mode
        // set motor other parameters
        // set motor PID parameters
        // set motors to listen to encoders

        // initialize motor pid timers and set callback function
        motors.vInitMotorPIDTimer();
    }

    // QRT_Sensor.attachObserver(line_follower)

    // set chassis to subscribe to rosserial cmd_vel message
}

void Chassis::meccanum_kinematics(Twist cmd_vel)
{
    _wheel_speeds[0] = (-1 * (chassis_length + chassis_width) * cmd_vel.w) + cmd_vel.x - cmd_vel.y;
    _wheel_speeds[1] = ((chassis_length + chassis_width) * cmd_vel.w) + cmd_vel.x + cmd_vel.y;
    _wheel_speeds[2] = ((chassis_length + chassis_width) * cmd_vel.w) + cmd_vel.x - cmd_vel.y;
    _wheel_speeds[3] = (-1 * (chassis_length + chassis_width) * cmd_vel.w) + cmd_vel.x + cmd_vel.y;
}

// use as ros subscriber callback function
void Chassis::chassisControl()
{
    // get Pose2D from rosserial

    // to compensate for off-centeredness, we add chassis velocity in the y direction
    _cmd_vel.y -= _line_following_gain * line_follower.getState();

    meccanum_kinematics(_cmd_vel);

    motors.Motor_setSpeed(_wheel_speeds[0], _wheel_speeds[1], _wheel_speeds[2], _wheel_speeds[3]);
    // Motor PID control loop handled in a separate RTOS thread
}

void Chassis::vInitChassisControlTimer()
{
    _loop_timer = xTimerCreate("Chassis Control Loop Timer", pdMS_TO_TICKS(1 / CONTROL_LOOP_FREQ), pdTRUE, (void *)0, Chassis::vChassisControlTimerCb);
    if (_loop_timer == NULL) while(1);
    else
        if( xTimerStart(_loop_timer, 0) != pdPASS ) while(1);
}

void Chassis::vChassisControlTimerCb(TimerHandle_t xTimer)
{
    Chassis::instance->chassisControl();
}