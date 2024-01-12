#ifndef ENCODER_H
#define ENCODER_H

#include "arduino_freertos.h"

#include "Subject.hpp"

enum Encoder_pin {
  PHASEA, PHASEB, HOME, INDEX, TRIGGER
};
typedef enum Encoder_pin Encoder_pin;

#define ENC_CTRL_HIRQ_MASK  (1 << 15)
#define ENC_CTRL_HIE_MASK   (1 << 14)
#define ENC_CTRL_HIP_MASK   (1 << 13)
#define ENC_CTRL_HNE_MASK   (1 << 12)
#define ENC_CTRL_SWIP_MASK  (1 << 11)
#define ENC_CTRL_REV_MASK   (1 << 10)
#define ENC_CTRL_PH1_MASK   (1 << 9)
#define ENC_CTRL_XIRQ_MASK  (1 << 8)
#define ENC_CTRL_XIE_MASK   (1 << 7)
#define ENC_CTRL_XIP_MASK   (1 << 6)
#define ENC_CTRL_XNE_MASK   (1 << 5)
#define ENC_CTRL_DIRQ_MASK  (1 << 4)
#define ENC_CTRL_DIE_MASK   (1 << 3)
#define ENC_CTRL_WDE_MASK   (1 << 2)
#define ENC_CTRL_CMPIRQ_MASK (1 << 2)
#define ENC_CTRL_CMPIE_MASK (1 << 0)

#define ENC_CTRL2_OUTCTL_MASK (1 << 9)
#define ENC_CTRL2_REVMOD_MASK (1 << 8)
#define ENC_CTRL2_ROIRQ_MASK  (1 << 7)
#define ENC_CTRL2_ROEI_MASK   (1 << 6)
#define ENC_CTRL2_RUIRQ_MASK  (1 << 5)
#define ENC_CTRL2_RUIE_MASK   (1 << 4)
#define ENC_CTRL2_DIR_MASK    (1 << 3)
#define ENC_CTRL2_MOD_MASK    (1 << 2)
#define ENC_CTRL2_UPDPOS_MASK (1 << 1)
#define ENC_CTRL2_UPDHLD_MASK (1 << 0)

#define ENC_FILT_PRSC(n)  (uint16_t)(((uint16_t)n << 13) & 0xE000)
#define ENC_FILT_CNT(n)   (uint16_t)(((uint16_t)n << 8) & 0x0700)
#define ENC_FILT_PER(n)   (uint16_t)(((uint16_t)n << 0) & 0x00FF)

typedef struct Encoder_state_t {
  int pos;
  int posDiff;
  int speed;
} Encoder_state_t;

class Encoder : public Subject<Encoder_state_t> {
public:
  Encoder();
  Encoder(uint8_t channel, uint8_t phaseA, uint8_t phaseB, uint8_t index, uint8_t home, uint8_t trigger, uint8_t pin_pus);
  ~Encoder();

  typedef struct ENC_Props {
    int hi;
    // pulse accumualtor functionality
    // glitch filter settings
    // timer prescalar
    // intial count
  } ENC_Props;

  void enableInterrupts();
  void disableInterrupts();
  void clearInterruptFlags();

  void setConfig(ENC_Props);
  void init();
  void setInitialCount(uint32_t initial_pos);
  uint16_t getPosition();
  uint16_t getPositionHold();
  uint16_t getPositionDifference();
  uint16_t getPositionDifferenceHold();
  uint16_t getRevolutions();
  uint16_t getRevolutionsHold();
  uint16_t getSpeed();
  bool self_test();
  
private:
  typedef struct {
		uint8_t			enc_ch;		
		volatile IMXRT_ENC_t* 	ENC;
		IRQ_NUMBER_t	interrupt;
		void     		(*isr)();
		uint16_t		phaseA;
		uint16_t		phaseB;
		uint16_t		index;
		uint16_t		home;
		uint16_t		trigger;
		volatile uint32_t *clock_gate_register;
		uint32_t 		clock_gate_mask;
	} ENC_Channel_t;

  static const uint8_t _channel_count;
  uint8_t _encoder_channel;
  static const ENC_Channel_t channel[];

  static Encoder *instances[5];

  static void isrEnc1();
	static void isrEnc2();
	static void isrEnc3();
	static void isrEnc4();
  void isr();

  void xbara_pin_map(uint16_t pin, Encoder_pin func, uint8_t PUS);
  void xbar_connect(unsigned int input, unsigned int output);

  // Counts are stored by the super class
};

#endif // !ENCODER_H
