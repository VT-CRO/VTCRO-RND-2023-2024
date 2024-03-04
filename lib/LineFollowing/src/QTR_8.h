#ifndef QTR_8_H
#define QTR_8_H

#include <stdint.h>
#include <arduino_freertos.h>
#include <atomic>

#define NUM_SENSORS 16

/*
typedef struct
{
  uint16_t* pins;
  uint16_t* data;
  uint16_t sampleSize;
  uint16_t arrayLength;
} QTR_t;
*/

class QTR_8 {
  public:
  QTR_8(uint16_t);
  bool initTask();
  double findPos();
  void sensorRead();
  void QTRloop();
  

  private:
  static void qtr8_task(void*);
  uint16_t sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15};
  /* pins for the sensor */

  uint16_t readings[NUM_SENSORS];
  /* stores the readings from each sensor */

  uint16_t sampleSize;
  /* the number of measurements taken when sensorRead is called */

  double error; //TODO: make this atomic!!
  /* the difference between the current position of the line and the center */
};

//QTR_t QTR_init(uint16_t pins[], uint16_t data[], uint16_t sampleSize, uint16_t arrayLength);

int changeBit (int initialVal);

void sensorRead(uint16_t sensorCount);

#endif