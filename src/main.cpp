#include "arduino_freertos.h"


FLASHMEM __attribute__((noinline)) void setup()
{
  // digitalWrite(arduino::LED_BUILTIN, arduino::HIGH);
  Serial.begin(9600);

  bool ok;
  while (1)
  {
    Serial.println("Hiii!");
    if (ok)
      digitalWrite(13, arduino::HIGH);
    else 
      digitalWrite(13, arduino::LOW);
    
    ok = !ok;

    delay(1000);
  }
  // setup the following tasks:
  // chassis drive task
  //    this task is responsible for controlling the chassis.
  //    this task has a queue where the user can send a command velocity
  //    upon receiving a new command velocity it updates its motors (pushes to their control loop queues)
  // line following task
  //    this task is responsible for line following
  //    runs periodically and works out a correction for the command velocity
  //    pushes a corrected command velocity to the chassis drive task
  // motor control task (optional)
  //    runs pid task for the motor and includes ramping and direction-switch braking
  //    has a queue that receives a command velocity
  // led blink task
  //    a task that blinks the led periodically
  //    has an error queue that receives holds the led on whenever it receives an error
  // application fsm task
  //    task for updating the application
  // servo smoothing task
  //    task for updating servo motion
  //    receives a goal position via queue
}

void loop()
{
  
}
