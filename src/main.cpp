#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <Observer.hpp>
#include <Subject.hpp>
#include <QDC.h>
#include <Chassis.h>

FLASHMEM __attribute__((noinline))
void setup() {
  Chassis chassis;
  chassis.vInitChassisControlTimer();

  vTaskStartScheduler();

  while(1)
  {
      Serial.println("Scheduler Failed! \n");
      Serial.flush();
      delay(1000);
  }
}

void loop() {
  delay(100);
}
