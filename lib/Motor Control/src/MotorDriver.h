//Seniors code


#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#define Right_Forward  6
#define Left_Forward  9

#define Right_Backward  5
#define Left_Backward 10

#include "arduino_freertos.h"

struct Movement{
  int Speed = 0;
  int RightForward = 0;
  int RightBack = 0;
  int LeftForward = 0;
  int LeftBack = 0;
};

class MotorDriver {

public:

void StopMove(Movement DoThis){
  double increment = .99;
  while (increment > 0){
  analogWrite(Right_Forward , increment*DoThis.RightForward*DoThis.Speed);
  analogWrite(Right_Backward, increment*DoThis.RightBack*DoThis.Speed);
  analogWrite(Left_Forward,increment*DoThis.LeftForward*DoThis.Speed);
  analogWrite(Left_Backward,increment*DoThis.LeftBack*DoThis.Speed);
  delay(1);
  increment = increment - .01;
  }
}

void StartMove(Movement DoThis){
  double increment = .01;
  while (increment < 1){
  analogWrite(Right_Forward , increment*DoThis.RightForward*DoThis.Speed);
  analogWrite(Right_Backward , increment*DoThis.RightBack*DoThis.Speed);
  analogWrite(Left_Forward , increment*DoThis.LeftForward*DoThis.Speed);
  analogWrite(Left_Backward , increment*DoThis.LeftBack*DoThis.Speed);
  delay(1);
  increment = increment + .01;
  Serial.print(increment);
  }
}

void FollowLine(Movement DoThis, double angle, double offset){


  analogWrite(Right_Forward, DoThis.Speed - 100*angle/90);//right forward
  analogWrite(Left_Forward, DoThis.Speed + 100*angle/90);//left forward

  Serial.println(angle);
  Serial.println(offset);

}

  int Speed = 125;
  int Offset = 0;

  Movement Left;
  Movement Right;
  Movement Forward;
  Movement Back;

  
  // Left.Speed = Speed;
  // Left.RightForward = 1;
  // Left.LeftBack = 1;

  // Right.Speed = Speed;
  // Right.RightBack = 1;
  // Right.LeftForward = 1;

  // Forward.Speed = Speed;
  // Forward.RightForward = 1;
  // Forward.LeftForward = 1;

  // Back.Speed = Speed;
  // Back.RightBack = 1;
  // Back.LeftBack = 1;
  
};

#endif