/////////////////////////////////////////////////////////////
// Author: Jayson De La Vega   R&D Team VT CRO
// filename: QDC.h
// Last Modified: 1/24/2024
// Description:  This file contains function and struct 
//               declarations for the Quadrature Decoder
//               peripheral of Teensy 4.x boards
//               
//               Adapted from:
//               https://github.com/mjs513/Teensy-4.x-Quad-Encoder-Library/tree/master
///////////////////////////////////////////////////////////// 
#ifndef QDC_H
#define QDC_H

#include <arduino_freertos.h>

#ifdef __cplusplus
extern "C" {
#endif

#define QDC_CTRL_HIRQ_MASK  (1 << 15)
#define QDC_CTRL_HIE_MASK   (1 << 14)
#define QDC_CTRL_HIP_MASK   (1 << 13)
#define QDC_CTRL_HNE_MASK   (1 << 12)
#define QDC_CTRL_SWIP_MASK  (1 << 11)
#define QDC_CTRL_REV_MASK   (1 << 10)
#define QDC_CTRL_PH1_MASK   (1 << 9)
#define QDC_CTRL_XIRQ_MASK  (1 << 8)
#define QDC_CTRL_XIE_MASK   (1 << 7)
#define QDC_CTRL_XIP_MASK   (1 << 6)
#define QDC_CTRL_XNE_MASK   (1 << 5)
#define QDC_CTRL_DIRQ_MASK  (1 << 4)
#define QDC_CTRL_DIE_MASK   (1 << 3)
#define QDC_CTRL_WDE_MASK   (1 << 2)
#define QDC_CTRL_CMPIRQ_MASK (1 << 2)
#define QDC_CTRL_CMPIE_MASK (1 << 0)

#define QDC_CTRL2_SABIRQ_MASK (1 <  12)
#define QDC_CTRL2_OUTCTL_MASK (1 << 9)
#define QDC_CTRL2_REVMOD_MASK (1 << 8)
#define QDC_CTRL2_ROIRQ_MASK  (1 << 7)
#define QDC_CTRL2_ROEI_MASK   (1 << 6)
#define QDC_CTRL2_RUIRQ_MASK  (1 << 5)
#define QDC_CTRL2_RUIE_MASK   (1 << 4)
#define QDC_CTRL2_DIR_MASK    (1 << 3)
#define QDC_CTRL2_MOD_MASK    (1 << 2)
#define QDC_CTRL2_UPDPOS_MASK (1 << 1)
#define QDC_CTRL2_UPDHLD_MASK (1 << 0)

#define QDC_FILT_PRSC(n)  (uint16_t)(((uint16_t)n << 13) & 0xE000)
#define QDC_FILT_CNT(n)   (uint16_t)(((uint16_t)n << 8) & 0x0700)
#define QDC_FILT_PER(n)   (uint16_t)(((uint16_t)n << 0) & 0x00FF)

#define QDC_CTRL_W1C_FLAGS (QDC_CTRL_HIRQ_MASK | QDC_CTRL_XIRQ_MASK | QDC_CTRL_DIRQ_MASK | QDC_CTRL_CMPIRQ_MASK)
#define QDC_CTRL2_W1C_FLAGS (QDC_CTRL2_SABIRQ_MASK | QDC_CTRL2_ROIRQ_MASK | QDC_CTRL2_RUIRQ_MASK)

// Config settings
#define DISABLE     0
#define ENABLE      1

#define QDC_NORMAL_MODE 0
#define QDC_PHASEA_MODE 1
#define QDC_RISING_EDGE_TRIG    1
#define QDC_FALLING_EDGE_TRIG   2

#define QDC_POS_MATCH_COMP_MODE 0
#define QDC_POS_MATCH_POS_MODE  1

#define QDC_REV_COND_INDEX  0   
#define QDC_REV_COND_MOD    1

// IRQ Disable masks
#define QDC_HOMETransistionEnable   (1 << 0)
#define QDC_INDEXPulseEnable        (1 << 1)
#define QDC_positionCompareEnable   (1 << 3)
#define QDC_positionROEnable        (1 << 5)
#define QDC_positionRUEnable        (1 << 6)

// IRQ Flag masks
#define QDC_HOMETransitionFlag      (1 << 0)
#define QDC_INDEXPulseFlag          (1 << 1)
#define QDC_positionCompareFlag     (1 << 3)
#define QDC_positionROFlag          (1 << 5)
#define QDC_positionRUFlag          (1 << 6)
#define QDC_lastDirectionFlag       (1 << 7)

// #define QDC_INDEX_IRQ(n)    (QDC_CTRL_XIRQ_MASK == QDC_CTRL_XIRQ_MASK & channel[n].ENC->CTRL) && (QDC_CTRL_XIE_MASK & channel[index].ENC->CTRL)
// #define QDC_RO_IRQ(n)       (QDC_CTRL2_ROIRQ_MASK == QDC_CTRL2_ROIRQ_MASK & channel[n].ENC->CTRL2)
// #define QDC_RU_IRQ(n)       (QDC_CTRL2_RUIRQ_MASK == QDC_CTRL2_RUIRQ_MASK & channel[n].ENC->CTRL2)
// #define QDC_HOME_IRQ(n)     (QDC_CTRL_HIRQ_MASK == QDC_CTRL_HIRQ_MASK & channel[n].ENC->CTRL) && (QDC_CTRL_HIE_MASK & channel[index].ENC->CTRL)
// #define QDC_CMP_IRQ(n)      (QDC_CTRL_CMPIRQ_MASK == QDC_CTRL_CMPIRQ_MASK & channel[n].ENC->CTRL)

typedef enum {
  PHASEA, PHASEB, HOME, INDEX, TRIGGER
} QDC_pin;

typedef struct {
    bool enableReverseDirection;
    bool decoderWorkMode;
    
    uint8_t HOMETriggerMode;
    uint8_t INDEXTriggerMode;
    bool clearCounter;
    bool clearHoldCounter;

    uint16_t filterCount;
    
    uint16_t filterSamplePeriod;

    bool positionMatchMode;

    bool positionCompareMode;

    uint32_t positionCompareVal;

    bool revolutionCountCondition;

    bool enableModuloCountMode;

    uint32_t positionModulusValue;

    uint32_t positionInitialVal;

    uint8_t positionROIE;
    uint8_t positionRUIE;

    bool IndexTrigger;
    bool HomeTrigger;
} QDC_config_t;

typedef struct {
    uint8_t			enc_ch;		
    volatile IMXRT_ENC_t* 	ENC;
    IRQ_NUMBER_t	interrupt;
    // void     		(*isr)();
    uint16_t		phaseA;
    uint16_t		phaseB;
    uint16_t		index;
    uint16_t		home;
    uint16_t		trigger;
    volatile uint32_t *clock_gate_register;
    uint32_t 		clock_gate_mask;
} QDC_channel_t;

typedef struct {
    uint8_t channel;

    QDC_config_t config;
} QDC_t;

QDC_t QDC_create();
void QDC_setConfig(QDC_t* qdc, QDC_config_t* config);
QDC_config_t QDC_makeConfig();
void QDC_mapDigitalPins(QDC_t * qdc, uint8_t phaseA, uint8_t phaseB, uint8_t index, uint8_t home, uint8_t trigger, uint8_t pin_pus);
void QDC_enableInterrupts(QDC_t * qdc);
void QDC_attachInterrupt(QDC_t * qdc, void (*isr)(void));
void QDC_disableInterrupts(QDC_t * qdc, uint32_t flag);
void QDC_clearInterruptFlags(QDC_t * qdc, uint32_t flag);
void QDC_init(QDC_t * qdc);
void QDC_setInitialCount(QDC_t * qdc, uint32_t initial_pos);
uint16_t QDC_getPosition(QDC_t * qdc);
uint16_t QDC_getPositionHold(QDC_t * qdc);
uint16_t QDC_getPositionDifference(QDC_t * qdc);
uint16_t QDC_getPositionDifferenceHold(QDC_t * qdc);
uint16_t QDC_getRevolutions(QDC_t * qdc);
uint16_t QDC_getRevolutionsHold(QDC_t * qdc);
uint32_t QDC_getCTRLIRQStatus(QDC_t * qdc);
uint32_t QDC_getCTRL2IRQStatus(QDC_t * qdc);

#ifdef __cplusplus
}
#endif

#endif // !QDC_H