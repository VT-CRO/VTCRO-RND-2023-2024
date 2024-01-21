#include "QTR_8.h"

uint16_t s0 = A0;
uint16_t s1 = A1;
uint16_t s2 = A2;
uint16_t s3 = A3;
uint16_t s4 = A4;
uint16_t s5 = A5;
uint16_t s6 = A6;
uint16_t s7 = A7;
uint16_t s8 = A8;
uint16_t s9 = A9;
uint16_t s10 = A10;
uint16_t s11 = A11;
uint16_t s12 = A12;
uint16_t s13 = A13;
uint16_t s14 = A14;
uint16_t s15 = A15;
uint16_t sensorPins[] = {s0, s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12, s13, s14, s15};
uint16_t readings[16];
QTR_t data{};

#define SENSOR_THRESHOLD 900

// TODO: Refactor as class
// TODO: create an initialization function to create the task. Look at chassis's initTask, chassisControl_task for examples. Don't worry about ros::Nodehandle  *nh because line follower doesnt need,
// TODO: edit line follower function calls in chassis control to match declarations here.
// TODO: initailize line followers in chassis contructor

double findPos() {

  uint16_t firstYellow = data.arrayLength;
  uint16_t lastYellow = data.arrayLength;
  for (int i = 0; i < data.arrayLength; i++) {
    if (data.data[i] < SENSOR_THRESHOLD) { //if i is yellow
      if (firstYellow == data.arrayLength) 
        firstYellow = i;
      
      lastYellow = i;
    }
  }
  
  if (firstYellow == data.arrayLength) {
    Serial.println(data.arrayLength);
    return data.arrayLength/2.0;
  }

  double center = ((double)(firstYellow + lastYellow)) / 2.0;
  return center;
}

void sensorRead(uint16_t sensorCount)
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

  for (uint8_t k = 0; k < sensorCount; k += 1)
      {
        data.data[k] = (data.data[k] + (4 >> 1)) /
          4;
      }
}

void loop() {
  // put your main code here, to run repeatedly:
  
  sensorRead(data.arrayLength);

  for (uint8_t i = 0; i < data.arrayLength; i++)
  {
    Serial.print("Value at ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(data.data[i]);
    
  }
  
  

 
  sensorRead(data.arrayLength);
  
  double position = findPos();
  Serial.print("Position: ");
  Serial.println(position);
  double error = position - data.arrayLength/2.0;
  Serial.print("Error amount: ");
  Serial.println(error);
  delay(1000);
}

QTR_t QTR_init(uint16_t pins[], uint16_t data[], uint16_t sampleSize, uint16_t arrayLength)
{
   QTR_t values = {pins, data, sampleSize, arrayLength};
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
