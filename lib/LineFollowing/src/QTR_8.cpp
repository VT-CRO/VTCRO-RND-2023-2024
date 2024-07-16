#include "QTR_8.h"
#include "HardwareDefs.h"

#define tskLINEFOLLOWING_PRIO 1 //TODO: review this value. not sure what the real value should be. -Robert
#define CONTROL_LOOP_PERIOD 50      //TODO: this value might need to change as well -Robert

//called by initTask; calls QTRloop repeatedly
void QTR_8_task(void * pvParameters)
{
  QTR_8_t* qtr_ptr = (QTR_8_t *)pvParameters;

  TickType_t ui32WakeTime = xTaskGetTickCount();

  while (1) {
    QTR_8_sensorRead(qtr_ptr);
    double err = QTR_8_findPos(qtr_ptr);

    // TODO: find a way to somehow send position to chassis
    
    xTaskDelayUntil(&ui32WakeTime, pdMS_TO_TICKS(CONTROL_LOOP_PERIOD));
  }
}

bool QTR_8_init(QTR_8_t* qtr_ptr, uint16_t* pins, uint16_t sampleSize)
{
  qtr_ptr->pins = pins;
  qtr_ptr->sampleSize = sampleSize;
  qtr_ptr->data = new uint16_t[NUM_SENSORS];

  BaseType_t ok;
  TaskHandle_t qtr_task_handle = NULL;

  ok = xTaskCreate(
      QTR_8_task,
      "qtr_task",
      50,
      (void *) qtr_ptr,
      tskIDLE_PRIORITY + tskLINEFOLLOWING_PRIO, 
      &qtr_task_handle);

  if (ok != pdPASS)
  {
      vTaskDelete(qtr_task_handle);
  }
  return (ok == pdPASS);
}

double QTR_8_findPos(QTR_8_t* qtr_ptr)
{
  uint16_t firstYellow = NUM_SENSORS;
  uint16_t lastYellow = NUM_SENSORS;

  for (int i = 0; i < NUM_SENSORS; i++) {
    bool isLine = qtr_ptr->data[i] < QTR_THRESHOLD;
    if (isLine) {
      if (firstYellow == NUM_SENSORS) {
        firstYellow = i;
      }
      
      lastYellow = i;
    }
  }
  
  if (firstYellow == NUM_SENSORS) {
    return NUM_SENSORS/2.0;
  }

  double center = ((double)(firstYellow + lastYellow)) / 2.0;
  return center;
}

void QTR_8_sensorRead(QTR_8_t* qtr_ptr)
{
  for (uint8_t r = 0; r < NUM_SENSORS; r += 1)
  {
    qtr_ptr->data[r] = 0;
  }

  for (uint8_t m = 0; m < qtr_ptr->sampleSize; m++)
  {
    for (uint8_t n = 0; n < NUM_SENSORS; n += 1)
    {
      // add the conversion result
      qtr_ptr->data[n] += analogRead(qtr_ptr->pins[n]);
    }
  }

  for (uint8_t k = 0; k < NUM_SENSORS; k += 1)
  {
    qtr_ptr->data[k] = (qtr_ptr->data[k] + (4 >> 1)) / qtr_ptr->sampleSize;
  }
}