#include "arduino_freertos.h"
#include "avr/pgmspace.h"
#include "MotorControl.h"

static void task1(void*) {
  while (true) {
    digitalWriteFast(arduino::LED_BUILTIN, arduino::LOW);
    vTaskDelay(pdMS_TO_TICKS(500));

    digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

static void task2(void*) {
  while (true) {
    Serial.println("TICK");
    vTaskDelay(pdMS_TO_TICKS(1'000));

    Serial.println("TOCK");
    vTaskDelay(pdMS_TO_TICKS(1'000));
  }
}

FLASHMEM __attribute__((noinline))
void setup() {
  // Serial.begin(115'200);
  // pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
  // digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);

  // delay(5'000);

  // if (CrashReport)  {
  //   Serial.print(CrashReport);
  //   Serial.println();
  //   Serial.flush();
  // }

  // Serial.println(PSTR("\r\nBooting FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"));

  // // create tasks
  // xTaskCreate(task1, "task1", 128, nullptr, 2, nullptr);
  // xTaskCreate(task2, "task2", 128, nullptr, 2, nullptr);

  // // start the scheduler
  // Serial.println("setup(): starting scheduler...");
  // Serial.flush();


   
  MotorControl m;
  m.Motor_pin_init();
  m.Motor_start();
  // while(1){
  //   Serial.print("The motors should have started\n");
  // }
 // m.Motor_setSpeed(127,127,127,127);
  
  vTaskStartScheduler();

}


void loop() {
  // put your main code here, to run repeatedly:
  
  // MotorControl m;
  // m.Motor_pin_init();
  // while(1){
  // m.Motor_start();
  // delay(1000);
  // m.stopMove();
}