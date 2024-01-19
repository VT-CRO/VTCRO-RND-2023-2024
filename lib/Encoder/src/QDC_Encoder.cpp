/////////////////////////////////////////////////////////////
// Author: Jayson De La Vega   R&D Team VT CRO
// filename: QDC.h
// Last Modified: 1/24/2024
// Description:  This file contains function and struct 
//               definitions for an Encoder based on the QDC
//               peripheral (only available on Teensy 4.x)
///////////////////////////////////////////////////////////// 
#include "QDC_Encoder.h"

static const uint8_t _channel_count;
static QDC_Encoder *instances[5];

static QDC_Encoder::QDC_isr isrs = {
    QDC_Encoder::isrEnc1,
    QDC_Encoder::isrEnc2,
    QDC_Encoder::isrEnc3,
    QDC_Encoder::isrEnc4
};