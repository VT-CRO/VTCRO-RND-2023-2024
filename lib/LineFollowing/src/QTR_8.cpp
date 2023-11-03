#include "QTR_8.h"
#include <arduino_freertos.h>


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
