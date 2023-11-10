#include "arduino_freertos.h"
#include "avr/pgmspace.h"
#include "QTR_8.h"

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



/*FLASHMEM __attribute__((noinline)) void setup() {
  //commenting everything out in setup right now, to test line following
  
  
  Serial.begin(115'200);
  pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
  digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);

  delay(5'000);

  if (CrashReport)  {
    Serial.print(CrashReport);
    Serial.println();
    Serial.flush();
  }

  Serial.println(PSTR("\r\nBooting FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"));

  // create tasks
  xTaskCreate(task1, "task1", 128, nullptr, 2, nullptr);
  xTaskCreate(task2, "task2", 128, nullptr, 2, nullptr);

  // start the scheduler
  Serial.println("setup(): starting scheduler...");
  Serial.flush();

  vTaskStartScheduler();
  
}
void loop() {
  // put your main code here, to run repeatedly:
}
*/

/* ----------------- COPIED OVER FROM QTR_8.cpp ----------------- */

uint16_t s0 = A0;
uint16_t s1 = A1;
uint16_t s2 = A2;
uint16_t s3 = A3;
uint16_t s4 = A4;
uint16_t s5 = A5;
uint16_t s6 = A6;
uint16_t s7 = A7;
uint16_t sensorPins[] = {s0, s1, s2, s3, s4, s5, s6, s7};
uint16_t readings[8];
QTR_t data{};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  data = QTR_init(sensorPins, readings, 4);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  sensorRead(data.data, 8);

  for (uint8_t i = 0; i < 8; i++)
  {
    Serial.print("Value at ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(data.data[i]);
    
  }
  
  delay(1000);
}

void sensorRead(uint16_t* sensorValues, int sensorCount)
{
    for (uint8_t r = 0; r < sensorCount; r += 1)
      {
        data.data[r] = 0;
      }

  for (uint8_t m = 0; m < 4; m++)
      {
        for (uint8_t n = 0; n < sensorCount; n += 1)
        {
          // add the conversion result
          data.data[n] += analogRead(data.pins[n]);
        }
      }

  for (uint8_t k = 0; k < 8; k += 1)
      {
        data.data[k] = (data.data[k] + (4 >> 1)) /
          4;
      }
}

QTR_t QTR_init(uint16_t pins[], uint16_t data[], uint16_t sampleSize)
{
   QTR_t values = {pins, data, sampleSize};
   return values;
}

//In line following algorithm
int changeBit (int initialVal)
{
    int changedBit = -1;

    if (initialVal >= 60)
    {
        changedBit = 0;
    }
    else
    {
        changedBit = 1;
    }

    return changedBit;
}