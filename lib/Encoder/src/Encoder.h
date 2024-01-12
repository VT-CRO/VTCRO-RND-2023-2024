#ifndef ENCODER_H
#define ENCODER_H

#include "arduino_freertos.h"

#include "Subject.hpp"

enum Encoder_pin {
  PHASEA, PHASEB, HOME, INDEX, TRIGGER
};
typedef enum Encoder_pin Encoder_pin;

class Encoder : public Subject<int> {
public:
  Encoder(uint8_t channel, uint8_t phaseA, uint8_t phaseB, uint8_t index, uint8_t home, uint8_t trigger, uint8_t pin_pus);
  Encoder(Encoder &&) = default;
  Encoder(const Encoder &) = default;
  Encoder &operator=(Encoder &&) = default;
  Encoder &operator=(const Encoder &) = default;
  ~Encoder();

  typedef struct ENC_Props {
    int hi;
  } ENC_Props;

  void enableInterrupts();
  void disableInterrupts();
  void clearStatusFlag();

  void setConfig(ENC_Props);
  void init();
  void write(uint32_t val);
  void getRevolutions();
  uint16_t getPosition();
  uint16_t getPositionDifference();

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
  static uint8_t _encoder_channel;
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
