#include <stdint.h>

typedef struct
{
  uint16_t* pins;
  uint16_t* data;
  uint16_t sampleSize;
  uint16_t arrayLength;
} QTR_t;

QTR_t QTR_init(uint16_t pins[], uint16_t data[], uint16_t sampleSize, uint16_t arrayLength);

int changeBit (int initialVal);

void sensorRead(uint16_t sensorCount);

class QTR {

  // ...

  uint16_t * pins;
  uint16_t numPins;

  void setPins(uint16_t * pins, uint16_t num_pins);

}