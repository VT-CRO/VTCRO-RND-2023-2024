#include <stdint.h>

typedef struct
{
  uint16_t* pins;
  uint16_t* data;
  uint16_t sampleSize;
} QTR_t;

QTR_t QTR_init(uint16_t pins[], uint16_t data[], uint16_t sampleSize);

int changeBit (int initialVal);

void sensorRead(uint16_t* sensorValues, int sensorCount);