/////////////////////////////////////////////////////////////
// Author: Jayson De La Vega   R&D Team VT CRO
// filename: QDC.cpp
// Last Modified: 1/24/2024
// Description:  This file contains function and struct 
//               definitions for the Quadrature Decoder
//               peripheral of Teensy 4.x boards
//               
//               Adapted from:
//               https://github.com/mjs513/Teensy-4.x-Quad-Encoder-Library/tree/master
//
//        *Note: This was originally written as a .c file
//               but imports didn't work for some reason
//               as a .c file, so I just kept it as .cpp
///////////////////////////////////////////////////////////// 
#include "QDC.h"

static void xbara_pin_map(QDC_t *qdc, uint16_t pin, QDC_pin func, uint8_t PUS);
static void xbar_connect(unsigned int input, unsigned int output);

static const QDC_channel_t channel[] = {
    {0, &IMXRT_ENC1, IRQ_ENC1, 66, 67, 68, 69, 70, &CCM_CCGR4, CCM_CCGR4_ENC1(CCM_CCGR_ON)}, // this is a dummy entry - use 1-4 for channels
    {1, &IMXRT_ENC1, IRQ_ENC1, 66, 67, 68, 69, 70, &CCM_CCGR4, CCM_CCGR4_ENC1(CCM_CCGR_ON)},
    {2, &IMXRT_ENC2, IRQ_ENC2, 71, 72, 73, 74, 75, &CCM_CCGR4, CCM_CCGR4_ENC2(CCM_CCGR_ON)},
    {3, &IMXRT_ENC3, IRQ_ENC3, 76, 77, 78, 79, 80, &CCM_CCGR4, CCM_CCGR4_ENC3(CCM_CCGR_ON)},
    {4, &IMXRT_ENC4, IRQ_ENC4, 81, 82, 83, 84, 85, &CCM_CCGR4, CCM_CCGR4_ENC4(CCM_CCGR_ON)}
};


QDC_t QDC_create()
{
    QDC_t qdc;

    return qdc;
}

void QDC_setConfig(QDC_t *qdc, QDC_config_t *config)
{
    qdc->config = *config;
}

QDC_config_t QDC_makeDefaultConfig()
{
    QDC_config_t config;

    config.enableReverseDirection = DISABLE;
    config.decoderWorkMode = DISABLE;
    config.HOMETriggerMode = DISABLE;
    config.INDEXTriggerMode = DISABLE;
    config.filterCount = 0;
    config.positionMatchMode = QDC_POS_MATCH_COMP_MODE;
    config.positionCompareMode = DISABLE;
    config.positionCompareVal = 0xffffffff;
    config.revolutionCountCondition = QDC_REV_COND_INDEX;
    config.enableModuloCountMode = DISABLE;
    config.positionModulusValue = 0;
    config.positionInitialVal = 0;
    config.positionROIE = DISABLE;
    config.positionRUIE = DISABLE;
    config.IndexTrigger = DISABLE;
    config.HomeTrigger = DISABLE;

    return config;
}

void QDC_mapDigitalPins(QDC_t *qdc, uint8_t phaseA, uint8_t phaseB, uint8_t index, uint8_t home, uint8_t trigger, uint8_t pin_pus)
{
    CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);

    xbara_pin_map(qdc, phaseA, PHASEA, pin_pus);
    xbara_pin_map(qdc, phaseB, PHASEB, pin_pus);
    // xbara_pin_map(qdc, index, INDEX, pin_pus);
    // xbara_pin_map(qdc, home, HOME, pin_pus);
    // xbara_pin_map(qdc, trigger, TRIGGER, pin_pus);
}

void QDC_enableInterrupts(QDC_t *qdc)
{
    uint32_t tmp16 = 0;

    NVIC_SET_PRIORITY(channel[qdc->channel].interrupt, 32);

    if (qdc->config.HomeTrigger != DISABLE)
        tmp16 |= QDC_CTRL_HIE_MASK;
    if (qdc->config.IndexTrigger != DISABLE)
        tmp16 |= QDC_CTRL_XIE_MASK;
    if (qdc->config.positionCompareMode == ENABLE)
        tmp16 |= QDC_CTRL_CMPIE_MASK;
    if (tmp16 != 0)
        channel[qdc->channel].ENC->CTRL = (channel[qdc->channel].ENC->CTRL & (uint16_t)(~QDC_CTRL_W1C_FLAGS)) | tmp16;
    tmp16 = 0U;
    if (qdc->config.positionROIE != DISABLE)
        tmp16 |= QDC_CTRL2_ROIRQ_MASK;
    if (qdc->config.positionRUIE != DISABLE)
        tmp16 |= QDC_CTRL2_RUIRQ_MASK;
    if (tmp16 != 0U)
        channel[qdc->channel].ENC->CTRL2 = (channel[qdc->channel].ENC->CTRL2 & (uint16_t)(~QDC_CTRL2_W1C_FLAGS)) & (uint16_t)(~tmp16);
}

void QDC_attachInterrupt(QDC_t *qdc, void(*isr)(void))
{
    attachInterruptVector(channel[qdc->channel].interrupt, isr);
}

void QDC_disableInterrupts(QDC_t *qdc, uint32_t flag)
{
    uint16_t tmp16 = 0U;
    if (QDC_HOMETransistionEnable == (QDC_HOMETransitionFlag & flag))
        tmp16 |= QDC_CTRL_HIE_MASK;
    if (QDC_INDEXPulseEnable == (QDC_INDEXPulseEnable & flag))
        tmp16 |= QDC_CTRL_XIE_MASK;
    if (QDC_positionCompareEnable == (QDC_positionCompareEnable & flag))
        tmp16 |= QDC_CTRL_CMPIE_MASK;
    if (0U != tmp16)
        channel[qdc->channel].ENC->CTRL = (uint16_t)(channel[qdc->channel].ENC->CTRL & (uint16_t)(~QDC_CTRL_W1C_FLAGS)) & (uint16_t)(~tmp16);

    tmp16 = 0U;
    if (QDC_positionROEnable == (QDC_positionROEnable & flag))
        tmp16 |= QDC_CTRL2_ROIRQ_MASK;
    if (QDC_positionRUEnable == (QDC_positionRUEnable & flag))
        tmp16 |= QDC_CTRL2_RUIRQ_MASK;
    if (0U != tmp16)
        channel[qdc->channel].ENC->CTRL = (uint16_t)(channel[qdc->channel].ENC->CTRL2 & (uint16_t)(~QDC_CTRL2_W1C_FLAGS)) & (uint16_t)(~tmp16);
}

void QDC_clearInterruptFlags(QDC_t *qdc, uint32_t flag)
{
    uint32_t tmp16 = 0U;

    if (QDC_HOMETransitionFlag == (QDC_HOMETransitionFlag & flag))
        tmp16 |= QDC_CTRL_HIRQ_MASK;
    if (QDC_INDEXPulseFlag == (QDC_INDEXPulseFlag & flag))
        tmp16 |= QDC_CTRL_XIRQ_MASK;
    if (QDC_positionCompareFlag == (QDC_positionCompareFlag & flag))
        tmp16 |= QDC_CTRL_CMPIRQ_MASK;
    if (0U != tmp16)
        channel[qdc->channel].ENC->CTRL = (channel[qdc->channel].ENC->CTRL & (uint16_t)(~QDC_CTRL_W1C_FLAGS)) | tmp16;

    tmp16 = 0U;
    if (QDC_positionROFlag == (QDC_positionROFlag & flag))
        tmp16 |= QDC_CTRL2_ROIRQ_MASK;
    if (QDC_positionRUFlag == (QDC_positionRUFlag & flag))
        tmp16 |= QDC_CTRL2_RUIRQ_MASK;
    if (0U != tmp16)
        channel[qdc->channel].ENC->CTRL2 = (channel[qdc->channel].ENC->CTRL2 & (uint16_t)(~QDC_CTRL2_W1C_FLAGS)) | tmp16;
}

void QDC_init(QDC_t *qdc)
{
    uint32_t tmp16;

    *channel[qdc->channel].clock_gate_register |= channel[qdc->channel].clock_gate_mask;

    tmp16 = channel[qdc->channel].ENC->CTRL & (uint16_t)(~(QDC_CTRL_W1C_FLAGS | QDC_CTRL_HIP_MASK | QDC_CTRL_HNE_MASK | QDC_CTRL_REV_MASK | QDC_CTRL_PH1_MASK | QDC_CTRL_XIP_MASK | QDC_CTRL_XNE_MASK | QDC_CTRL_WDE_MASK));

    if (qdc->config.HOMETriggerMode != DISABLE)
    {
        tmp16 |= QDC_CTRL_HIP_MASK;
        if (qdc->config.HOMETriggerMode == QDC_FALLING_EDGE_TRIG)
            tmp16 |= QDC_CTRL_HNE_MASK;
    }

    if (qdc->config.enableReverseDirection)
        tmp16 |= QDC_CTRL_REV_MASK;
    if (qdc->config.decoderWorkMode == QDC_PHASEA_MODE)
        tmp16 |= QDC_CTRL_PH1_MASK;
    if (qdc->config.INDEXTriggerMode != DISABLE)
    {
        tmp16 |= QDC_CTRL_XIP_MASK;
        if (QDC_FALLING_EDGE_TRIG == qdc->config.INDEXTriggerMode)
            tmp16 |= QDC_CTRL_XNE_MASK;
    }

    channel[qdc->channel].ENC->CTRL = tmp16;

    channel[qdc->channel].ENC->FILT = QDC_FILT_CNT(qdc->config.filterCount) | QDC_FILT_PER(qdc->config.filterSamplePeriod);

    tmp16 = channel[qdc->channel].ENC->CTRL2 & (uint16_t)(~(QDC_CTRL2_W1C_FLAGS | QDC_CTRL2_OUTCTL_MASK | QDC_CTRL2_REVMOD_MASK | QDC_CTRL2_MOD_MASK | QDC_CTRL2_UPDPOS_MASK | QDC_CTRL2_UPDHLD_MASK | QDC_CTRL2_ROIRQ_MASK | QDC_CTRL2_RUIRQ_MASK));

    if (qdc->config.positionMatchMode == QDC_POS_MATCH_POS_MODE)
        tmp16 |= QDC_CTRL2_OUTCTL_MASK;
    if (qdc->config.revolutionCountCondition == QDC_REV_COND_MOD)
        tmp16 |= QDC_CTRL2_REVMOD_MASK;
    if (qdc->config.enableModuloCountMode)
    {
        tmp16 |= QDC_CTRL2_MOD_MASK;
        channel[qdc->channel].ENC->UMOD = (uint16_t)(qdc->config.positionModulusValue >> 16);
        channel[qdc->channel].ENC->LMOD = (uint16_t)(qdc->config.positionModulusValue);
    }
    if (qdc->config.clearCounter)
        tmp16 |= QDC_CTRL2_UPDPOS_MASK;
    if (qdc->config.clearHoldCounter)
        tmp16 |= QDC_CTRL2_UPDHLD_MASK;

    channel[qdc->channel].ENC->CTRL2 = tmp16;

    channel[qdc->channel].ENC->UCOMP = (uint16_t)(qdc->config.positionCompareVal >> 16);
    channel[qdc->channel].ENC->LCOMP = (uint16_t)(qdc->config.positionCompareVal);

    channel[qdc->channel].ENC->UINIT = (uint16_t)(qdc->config.positionInitialVal >> 16);
    channel[qdc->channel].ENC->LINIT = (uint16_t)(qdc->config.positionInitialVal);
}

void QDC_setInitialCount(QDC_t *qdc, uint32_t initial_pos)
{
    channel[qdc->channel].ENC->UINIT = (uint16_t)(initial_pos >> 16);
    channel[qdc->channel].ENC->LINIT = (uint16_t)(initial_pos);
}

uint16_t QDC_getPosition(QDC_t *qdc)
{
    uint32_t pos = 0;
    pos |= channel[qdc->channel].ENC->UPOS;
    pos <<= 16;
    pos |= channel[qdc->channel].ENC->LPOS;

    return pos;
}

uint16_t QDC_getPositionHold(QDC_t *qdc)
{
    uint32_t pos = 0;
    pos |= channel[qdc->channel].ENC->UPOSH;
    pos <<= 16;
    pos |= channel[qdc->channel].ENC->LPOS;

    return pos;
}

uint16_t QDC_getPositionDifference(QDC_t *qdc) { return channel[qdc->channel].ENC->POSD; }

uint16_t QDC_getPositionDifferenceHold(QDC_t *qdc) { return channel[qdc->channel].ENC->POSDH; }

uint16_t QDC_getRevolutions(QDC_t *qdc) { return channel[qdc->channel].ENC->REV; }

uint16_t QDC_getRevolutionsHold(QDC_t *qdc) { return channel[qdc->channel].ENC->REVH; }

void xbara_pin_map(QDC_t *qdc, uint16_t pin, QDC_pin func, uint8_t PUS)
{
    int pin_idx = 0;

    for (; pin_idx < count_pin_to_xbar_info; ++pin_idx)
    {
        if (pin_to_xbar_info[pin_idx].pin == pin)

            break;
    }

    IOMUXC_XBAR1_IN02_SELECT_INPUT;

    // The pin is not a valid xbar pin
    if (pin_idx == count_pin_to_xbar_info)
        return;

    *(portConfigRegister(pin)) = pin_to_xbar_info[pin_idx].mux_val;

    // TODO: set PUS configuration
    if (PUS == 0)
        *(portControlRegister(pin)) = 0x10B0;
    else
        *(portControlRegister(pin)) = 0x1f038;

    // MUX xbar
    if (pin_to_xbar_info[pin_idx].select_input_register)
        *(pin_to_xbar_info[pin_idx].select_input_register) = pin_to_xbar_info[pin_idx].select_val;

    switch (func)
    {
    case PHASEA:
        xbar_connect(pin_to_xbar_info[pin_idx].xbar_in_index, channel[qdc->channel].phaseA);
        break;
    case PHASEB:
        xbar_connect(pin_to_xbar_info[pin_idx].xbar_in_index, channel[qdc->channel].phaseB);
        break;
    case HOME:
        xbar_connect(pin_to_xbar_info[pin_idx].xbar_in_index, channel[qdc->channel].home);
        break;
    case INDEX:
        xbar_connect(pin_to_xbar_info[pin_idx].xbar_in_index, channel[qdc->channel].index);
        break;
    case TRIGGER:
        xbar_connect(pin_to_xbar_info[pin_idx].xbar_in_index, channel[qdc->channel].trigger);
        break;
    default:
        break;
    }
}

void xbar_connect(unsigned int input, unsigned int output)
{
    if (input >= 88)
        return;
    if (output >= 132)
        return;
#if 1
    volatile uint16_t *xbar = &XBARA1_SEL0 + (output / 2);
    uint16_t val = *xbar;
    if (!(output & 1))
    {
        val = (val & 0xFF00) | input;
    }
    else
    {
        val = (val & 0x00FF) | (input << 8);
    }
    *xbar = val;
#else
    // does not work, seems 8 bit access is not allowed
    volatile uint8_t *xbar = (volatile uint8_t *)XBARA1_SEL0;
    xbar[output] = input;
#endif
}

uint32_t QDC_getCTRLIRQStatus(QDC_t * qdc)
{
    uint32_t enabledInterrupts = channel[qdc->channel].ENC->CTRL | QDC_CTRL_XIE_MASK | QDC_CTRL_HIE_MASK | QDC_CTRL_CMPIE_MASK;
    uint32_t triggeredInterrupts = channel[qdc->channel].ENC->CTRL | QDC_CTRL_XIE_MASK | QDC_CTRL_HIRQ_MASK | QDC_CTRL_CMPIRQ_MASK;
    return triggeredInterrupts & (enabledInterrupts << 1);
}

uint32_t QDC_getCTRL2IRQStatus(QDC_t * qdc)
{
    uint32_t enabledInterrupts = channel[qdc->channel].ENC->CTRL2 | QDC_CTRL2_ROEI_MASK | QDC_CTRL2_RUIRQ_MASK;
    uint32_t triggeredInterrupts = channel[qdc->channel].ENC->CTRL2 | QDC_CTRL2_ROIRQ_MASK | QDC_CTRL2_RUIRQ_MASK;
    return triggeredInterrupts & (enabledInterrupts << 1);
}