/*
 NMEA2000_STM32.cpp

 Inherited NMEA2000 object for the STM32F105 internal CAN

 use with:
 - STM32 HAL for your MCU
 - STM32_CAN lib
 - NMEA2000 lib

 Only use with STM32 HAL for your MCU!


 Copyright (c) 2022 Minos Eigenheer

 Permission is hereby granted, free of charge, to any person obtaining a copy of
 this software and associated documentation files (the "Software"), to deal in
 the Software without restriction, including without limitation the rights to
 use,
 copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
 Software, and to permit persons to whom the Software is furnished to do so,
 subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED,
 INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 COPYRIGHT
 HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 ACTION OF
 CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE
 OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 */

#include "NMEA2000_STM32.hpp"


//*****************************************************************************
tNMEA2000_STM32::tNMEA2000_STM32(CAN_HandleTypeDef *_canBus) :
		tSTM32_CAN(_canBus, tSTM32_CAN::CAN250kbit), // NMEA2000 CAN is allways 250kbit
		tNMEA2000() {

}

tNMEA2000_STM32::~tNMEA2000_STM32() {
}

// forwarding of tNMEA2000 virtual functions to the corresponding functions in tSTM32_CAN base class
void tNMEA2000_STM32::InitCANFrameBuffers() {
	return tSTM32_CAN::InitCANFrameBuffers();
}

bool tNMEA2000_STM32::CANOpen() {
	bool ret;
	ret = tSTM32_CAN::CANOpen();

	// only accept extended frames on NMEA2000 CAN bus
	if (ret == HAL_OK) {
		ret = tSTM32_CAN::SetCANFilter( true, 0, 0x00000000, 0x00000000 );
	}
	return ret;
}

bool tNMEA2000_STM32::CANSendFrame(unsigned long id, unsigned char len, const unsigned char* buf, bool wait_sent) {
	return tSTM32_CAN::CANSendFrame(id, len, buf, wait_sent);
}

bool tNMEA2000_STM32::CANGetFrame(unsigned long& id, unsigned char& len, unsigned char* buf) {
	return tSTM32_CAN::CANGetFrame(id, len, buf);
}

// *****************************************************************************
//	Other 'Bridge' functions

void delay(const uint32_t ms) {
	HAL_Delay(ms);
};

uint32_t millis(void) {
    return HAL_GetTick();
};

// N2kStream is using write() function
int write(char *str, int len) {
	return _write(0, str, len);
}

