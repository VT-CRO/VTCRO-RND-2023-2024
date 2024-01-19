#ifndef CHASSIS_H
#define CHASSIS_H

#include "MotorControl.h"
#include "Observer.hpp"

#define NUM_MOTORS 4
#define CONTROL_LOOP_FREQ 50

/*
 * This class specifies the class interface for the chassis.
 * The chassis with the corresponding wheel numbers is as such:
 * 
 *                 ^  +x
 *                 |
 *          [1]---------[2]
 *           |   Front   |
 *   +y <--- |     x z+  |
 *           |           |
 *          [4]---------[3]
 */ 
class Chassis {
public:
  Chassis();
  Chassis(Chassis &&) = default;
  Chassis(const Chassis &) = default;
  Chassis &operator=(Chassis &&) = default;
  Chassis &operator=(const Chassis &) = default;
  ~Chassis();

  void init();
  void chassisControl();
  void vInitChassisControlTimer();

private:
  // Motor control
  MotorControl motors;
  int _wheel_speeds[NUM_MOTORS];

  // Line following
  Observer<int> line_follower;

  // gain for linefollowing process
  int _line_following_gain;

  // Chassis dimensions
  double chassis_length, chassis_width;

  struct Twist {
    int x; int y; int w;
  };
  typedef struct Twist Twist;

  Twist _cmd_vel;

  // feed in rosserial cmd_vel message into this function to get wheel speeds
  void meccanum_kinematics(Twist cmd_vel);

  // use as a callback function for the linefollowing sensor
  void update_center_err(int err);

  static void vChassisControlTimerCb(TimerHandle_t xTimer);
  TimerHandle_t _loop_timer;
  static Chassis* instance;
};

#endif // !CHASSIS_H
