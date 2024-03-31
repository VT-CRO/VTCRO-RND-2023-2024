#ifndef QTR_8_H
#define QTR_8_H

#include <stdint.h>
#include <arduino_freertos.h>
#include <atomic>

#define NUM_SENSORS 16

typedef struct
{
  uint16_t* pins;
  uint16_t* data;
  uint16_t sampleSize;

  void (*qtr_cb)(uint16_t);
} QTR_8_t;

bool QTR_8_init(QTR_8_t* qtr_ptr, uint16_t* pins, uint16_t sampleSize);
double QTR_8_findPos(QTR_8_t* qtr_ptr);
void QTR_8_sensorRead(QTR_8_t* qtr_ptr);

#endif