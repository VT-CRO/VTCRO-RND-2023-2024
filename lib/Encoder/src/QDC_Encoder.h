/////////////////////////////////////////////////////////////
// Author: Jayson De La Vega   R&D Team VT CRO
// filename: QDC_Encoder.h
// Last Modified: 1/24/2024
// Contact: jaysond21@vt.edu
// Description:  This file contains function and struct 
//               declarations for an Encoder based on the QDC
//               peripheral (only available on Teensy 4.x)
///////////////////////////////////////////////////////////// 
#ifndef QDC_ENCODER_H
#define QDC_ENCODER_H

#include "arduino_freertos.h"
#include "QDC.h"

#define MAX_NUM_ENCODER_INSTANCES 4

typedef struct Encoder_state_t {
  int pos;
  int posDiff;
} QDC_Encoder_state_t;

/*
 * This class is responsible for storing and configuring an
 * encoder based on the QDC peripheral on the Teensy microcontroller.
 * 
 * Note: This class is structured in a way that would make it easy to refactor
 * it to an abstract interface if different types of encoders need
 * to be implemented.
 * 
 */
class QDC_Encoder {
public:
  typedef void(*QDC_isr)();

  QDC_Encoder(uint8_t channel, uint8_t phaseA, uint8_t phaseB, uint8_t index, uint8_t home, uint8_t trigger, uint8_t pin_pus);
  ~QDC_Encoder();

  void setConfig(QDC_config_t* config);

  void enableInterrupts();
  void disableInterrupts(uint8_t flag);
  void clearInterruptFlags(uint8_t flag);

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

  // TODO: Figure out how encoders are gonna interface with motors

  static void isrEnc1();
	static void isrEnc2();
	static void isrEnc3();
	static void isrEnc4();
  
private:
  
  static QDC_Encoder *instances[5];

  static QDC_isr isrs[];

  QDC_t qdc;
  uint16_t indexCounter;
  uint16_t homeCounter;
  uint16_t compareValueFlag;

  void isr();
};

#endif // !QDC_ENCODER_H
