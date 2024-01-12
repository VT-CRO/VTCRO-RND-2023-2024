#include "Encoder.h"
#include "arduino_freertos.h"

void Encoder::init()
{
    // enable bus clock
    // map digital pin to encoder pin

    // enable ccm
}

void xbara_pin_map(uint16_t pin, uint16_t func, uint8_t pus)
{
    digital_pin_bitband_and_config_table_struct *p;

    p = digital_pin_to_info_PGM + pin;
}

// map input to output
void xbar_connect(uint16_t in, uint16_t out)
{
    if (in >= 88) return;
    if (out >= 132) return;
    volatile uint16_t *xbar = &XBARA1_SEL0 + (out / 2);
    if (out % 2)
    {
        *xbar = (*xbar & 0xFF00) | (in << 8);
    }
    else
        *xbar = (*xbar & 0x00FF) | in;
}