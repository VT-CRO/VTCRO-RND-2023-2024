#include "Encoder.h"

const Encoder::ENC_Channel_t Encoder::channel[] = {	
	{0, &IMXRT_ENC1, IRQ_ENC1, isrEnc1, 66, 67, 68, 69, 70,&CCM_CCGR4,CCM_CCGR4_ENC1(CCM_CCGR_ON)},  //this is a dummy entry - use 1-4 for channels
	{1, &IMXRT_ENC1, IRQ_ENC1, isrEnc1, 66, 67, 68, 69, 70,&CCM_CCGR4,CCM_CCGR4_ENC1(CCM_CCGR_ON)},
	{2, &IMXRT_ENC2, IRQ_ENC2, isrEnc2, 71, 72, 73, 74, 75,&CCM_CCGR4,CCM_CCGR4_ENC2(CCM_CCGR_ON)},
	{3, &IMXRT_ENC3, IRQ_ENC3, isrEnc3, 76, 77, 78, 79, 80,&CCM_CCGR4,CCM_CCGR4_ENC3(CCM_CCGR_ON)},
	{4, &IMXRT_ENC4, IRQ_ENC4, isrEnc4, 81, 82, 83, 84, 85,&CCM_CCGR4,CCM_CCGR4_ENC4(CCM_CCGR_ON)}
};
const uint8_t Encoder::_channel_count =  (sizeof(Encoder::channel)/sizeof(Encoder::channel[0]));

Encoder *Encoder::instances[5];

Encoder::Encoder(uint8_t channel, uint8_t phaseA, uint8_t phaseB, uint8_t index, uint8_t home, uint8_t trigger, uint8_t pin_pus)
{
    // check if encoder channel is valid
    if (channel < _channel_count) {
        // enable xbar clock
        CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);

        // map digital pin to encoder pin
        xbara_pin_map(phaseA,  PHASEA,  pin_pus);
        xbara_pin_map(phaseB,  PHASEB,  pin_pus);
        xbara_pin_map(index,   INDEX,   pin_pus);
        xbara_pin_map(home,    HOME,    pin_pus);
        xbara_pin_map(trigger, TRIGGER, pin_pus);
    }
}


void Encoder::xbara_pin_map(uint16_t pin, Encoder_pin func, uint8_t pus)
{
    int pin_idx = 0;

    for (; pin_idx < count_pin_to_xbar_info; ++pin_idx)
    {
        if (pin_to_xbar_info[pin_idx].pin == pin)

            break;
    }

    IOMUXC_XBAR1_IN02_SELECT_INPUT;

    // The pin is not a valid xbar pin
    if (pin_idx == count_pin_to_xbar_info) return;

    *(portConfigRegister(pin)) = pin_to_xbar_info[pin_idx].mux_val;

    // TODO: set PUS configuration

    // MUX xbar
    if (pin_to_xbar_info[pin_idx].select_input_register)
        *(pin_to_xbar_info[pin_idx].select_input_register) = pin_to_xbar_info[pin_idx].select_val;

    switch(func)
    {
        case PHASEA:
            xbar_connect(pin_to_xbar_info[pin_idx].xbar_in_index, channel[_encoder_channel].phaseA);
            break;
        case PHASEB:
            xbar_connect(pin_to_xbar_info[pin_idx].xbar_in_index, channel[_encoder_channel].phaseB);
            break;
        case HOME:
            xbar_connect(pin_to_xbar_info[pin_idx].xbar_in_index, channel[_encoder_channel].home);
            break;
        case INDEX:
            xbar_connect(pin_to_xbar_info[pin_idx].xbar_in_index, channel[_encoder_channel].index);
            break;
        case TRIGGER:
            xbar_connect(pin_to_xbar_info[pin_idx].xbar_in_index, channel[_encoder_channel].trigger);
            break;
        default:
            break;
    }
}

// map input to output
void Encoder::xbar_connect(unsigned int input, unsigned int output)
{
	if (input >= 88) return;
	if (output >= 132) return;
#if 1
	volatile uint16_t *xbar = &XBARA1_SEL0 + (output / 2);
	uint16_t val = *xbar;
	if (!(output & 1)) {
		val = (val & 0xFF00) | input;
	} else {
		val = (val & 0x00FF) | (input << 8);
	}
	*xbar = val;
#else
	// does not work, seems 8 bit access is not allowed
	volatile uint8_t *xbar = (volatile uint8_t *)XBARA1_SEL0;
	xbar[output] = input;
#endif
}

void Encoder::isrEnc1()
{

}

void Encoder::isrEnc2()
{

}

void Encoder::isrEnc3()
{

}

void Encoder::isrEnc4()
{

}

void isr()
{

}

void Encoder::enableInterrupts()
{
    // TODO
}

void Encoder::disableInterrupts()
{
    // TODO
}

void Encoder::clearInterruptFlags()
{
    // TODO
}

void Encoder::setConfig(ENC_Props props)
{
    // TODO
}

void Encoder::init()
{
    // TODO
}

void Encoder::setInitialCount(uint32_t initial_pos)
{
    channel[_encoder_channel].ENC->UINIT = (uint16_t)(initial_pos >> 16);
    channel[_encoder_channel].ENC->LINIT = (uint16_t)(initial_pos);
}

uint16_t Encoder::getPosition()
{
    uint32_t pos = 0;
    pos |= channel[_encoder_channel].ENC->UPOS;
    pos <<= 16;
    pos |= channel[_encoder_channel].ENC->LPOS;

    return pos;
}

uint16_t Encoder::getPositionHold()
{
    uint32_t pos = 0;
    pos |= channel[_encoder_channel].ENC->UPOSH;
    pos <<= 16;
    pos |= channel[_encoder_channel].ENC->LPOS;

    return pos;
}

uint16_t Encoder::getPositionDifference() { return channel[_encoder_channel].ENC->POSD; }

uint16_t Encoder::getPositionDifferenceHold() { return channel[_encoder_channel].ENC->POSDH; }

uint16_t Encoder::getRevolutions(){ return channel[_encoder_channel].ENC->REV; }

uint16_t Encoder::getRevolutionsHold() { return channel[_encoder_channel].ENC->REVH; }

uint16_t Encoder::getSpeed()
{
    // TODO
    return 0;
}

bool Encoder::self_test()
{
    // TODO
    return false;
}