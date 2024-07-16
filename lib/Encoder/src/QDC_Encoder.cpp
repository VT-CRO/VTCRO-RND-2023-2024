/////////////////////////////////////////////////////////////
// Author: Jayson De La Vega   R&D Team VT CRO
// filename: QDC_Encoder.cpp
// Last Modified: 1/24/2024
// Description:  This file contains function and struct 
//               definitions for an Encoder based on the QDC
//               peripheral (only available on Teensy 4.x)
///////////////////////////////////////////////////////////// 
#include "QDC_Encoder.h"

QDC_Encoder* QDC_Encoder::instances[5];

QDC_Encoder::QDC_isr QDC_Encoder::isrs[] = {
    QDC_Encoder::isrEnc1,
    QDC_Encoder::isrEnc2,
    QDC_Encoder::isrEnc3,
    QDC_Encoder::isrEnc4
};

QDC_Encoder::QDC_Encoder(uint8_t channel, uint8_t phaseA, uint8_t phaseB, uint8_t index, uint8_t home, uint8_t trigger, uint8_t pin_pus)
{
    qdc = QDC_create();
    QDC_mapDigitalPins(&qdc, phaseA, phaseB, index, home, trigger, pin_pus);

    if (channel > 0 && channel <= MAX_NUM_ENCODER_INSTANCES)
    {
        qdc.channel = channel;
        instances[channel] = this;
    } else {
        return;
    }
}

QDC_Encoder::~QDC_Encoder()
{
    instances[qdc.channel] = nullptr;
}

void QDC_Encoder::enableInterrupts()
{
    QDC_enableInterrupts(&qdc);
}

void QDC_Encoder::disableInterrupts(uint8_t flag)
{
    QDC_disableInterrupts(&qdc, flag);
}

void QDC_Encoder::clearInterruptFlags(uint8_t flag)
{
    QDC_clearInterruptFlags(&qdc, flag);
    QDC_attachInterrupt(&qdc, isrs[qdc.channel]);
}

void QDC_Encoder::init()
{
    QDC_init(&qdc);
}

void QDC_Encoder::setConfig(QDC_config_t* config)
{
    QDC_setConfig(&qdc, config);
}

void QDC_Encoder::setInitialCount(uint32_t initial_pos)
{
    QDC_setInitialCount(&qdc, initial_pos);
}

uint16_t QDC_Encoder::getPosition()
{
    return QDC_getPosition(&qdc);
}

uint16_t QDC_Encoder::getPositionHold()
{
    return QDC_getPositionHold(&qdc);
}

uint16_t QDC_Encoder::getPositionDifference()
{
    return QDC_getPositionDifference(&qdc);
}

uint16_t QDC_Encoder::getPositionDifferenceHold()
{
    return QDC_getPositionDifferenceHold(&qdc);
}

uint16_t QDC_Encoder::getRevolutions()
{
    return QDC_getRevolutions(&qdc);
}

uint16_t QDC_Encoder::getRevolutionsHold()
{
    return QDC_getRevolutionsHold(&qdc);
}

uint16_t QDC_Encoder::getSpeed()
{
    // TODO: Implement speed calculation algorithm
    
    return 0;
}

bool QDC_Encoder::self_test()
{
    return false;
}

void QDC_Encoder::isrEnc1()
{
    instances[1]->isr();
}

void QDC_Encoder::isrEnc2()
{
    instances[2]->isr();
}

void QDC_Encoder::isrEnc3()
{
    instances[3]->isr();
}

void QDC_Encoder::isrEnc4()
{
    instances[4]->isr();
}
  
void QDC_Encoder::isr()
{
    uint32_t ctrl_irq_status = QDC_getCTRLIRQStatus(&qdc);

    if (ctrl_irq_status | QDC_CTRL_XIRQ_MASK)
    {
        indexCounter++;
        QDC_clearInterruptFlags(&qdc, QDC_INDEXPulseFlag);

        uint32_t ctrl2_irq_status = QDC_getCTRL2IRQStatus(&qdc);

        if (ctrl2_irq_status | QDC_CTRL2_ROIRQ_MASK)
            QDC_clearInterruptFlags(&qdc, QDC_CTRL2_ROIRQ_MASK);
        if (ctrl2_irq_status | QDC_CTRL2_RUIRQ_MASK)
            QDC_clearInterruptFlags(&qdc, QDC_CTRL2_RUIRQ_MASK);
    }

    if (ctrl_irq_status | QDC_CTRL_HIRQ_MASK)
    {
        homeCounter++;
        QDC_clearInterruptFlags(&qdc, QDC_HOMETransitionFlag);
    }
    if (ctrl_irq_status | QDC_CTRL_CMPIRQ_MASK)
    {
        compareValueFlag = 1;
        QDC_disableInterrupts(&qdc, QDC_positionCompareEnable);
        QDC_clearInterruptFlags(&qdc, QDC_positionCompareFlag);
    }
}