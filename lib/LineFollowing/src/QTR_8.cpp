#include "QTR_8.h"
/*
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
*/

#define SENSOR_THRESHOLD 900
#define tskLINEFOLLOWING_PRIORITY 1 //TODO: review this value. not sure what the real value should be. -Robert
#define CONTROL_LOOP_PERIOD 50      //TODO: this value might need to change as well -Robert


// TODO: Refactor as class (DONE)
// TODO: create an initialization function to create the task. Look at chassis's initTask, chassisControl_task for examples. Don't worry about ros::Nodehandle  *nh because line follower doesnt need,
// (done)
// TODO: edit line follower function calls in chassis control to match declarations here.
// TODO: initailize line followers in chassis contructor



//constructor, sets the sampleSize
QTR_8::QTR_8(uint16_t sampleSize) : sampleSize (sampleSize)
{}

//called once, to initialize task of line following (finding error)
bool QTR_8::initTask()
{
  // initialize motor tasks
  if (xTaskCreate(QTR_8::qtr8_task, "line following task", 100, this, tskIDLE_PRIORITY + tskLINEFOLLOWING_PRIORITY, NULL) != pdTRUE)
    return 1;
  return 0;
}

//called by initTask; calls QTRloop repeatedly
void QTR_8::qtr8_task(void * pvParameters) {
  QTR_8* instance = (QTR_8 *)pvParameters;

  TickType_t ui32WakeTime = xTaskGetTickCount();

  while (1) {
      instance->QTRloop();
      
      xTaskDelayUntil(&ui32WakeTime, pdMS_TO_TICKS(CONTROL_LOOP_PERIOD));
  }
}


double QTR_8::findPos() {

  uint16_t firstYellow = NUM_SENSORS;
  uint16_t lastYellow = NUM_SENSORS;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (readings[i] < SENSOR_THRESHOLD) { //if i is yellow
      if (firstYellow == NUM_SENSORS) 
        firstYellow = i;
      
      lastYellow = i;
    }
  }
  
  if (firstYellow == NUM_SENSORS) {
    Serial.println(NUM_SENSORS);
    return NUM_SENSORS/2.0;
  }

  double center = ((double)(firstYellow + lastYellow)) / 2.0;
  return center;
}

void QTR_8::sensorRead()
{
    for (uint8_t r = 0; r < NUM_SENSORS; r += 1)
      {
        readings[r] = 0;
      }

  for (uint8_t m = 0; m < 4; m++)
      {
        for (uint8_t n = 0; n < NUM_SENSORS; n += 1)
        {
          // add the conversion result
          readings[n] += analogRead(sensorPins[n]);
        }
      }

  for (uint8_t k = 0; k < NUM_SENSORS; k += 1)
      {
        readings[k] = (readings[k] + (4 >> 1)) /
          4;
      }
}

void QTR_8::QTRloop() {
  // put your main code here, to run repeatedly:
  
  sensorRead();

  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print("Value at ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(readings[i]);
    
  }
 
  sensorRead();
  
  double position = findPos();
  Serial.print("Position: ");
  Serial.println(position);
  error = position - NUM_SENSORS/2.0;
  Serial.print("Error amount: ");
  Serial.println(error);
  delay(1000);
}

