/*
 NMEA2000_STM32.hpp

 Inherited NMEA2000 object for the STM32F105 internal CAN

 use with:
 - STM32 HAL for your MCU
 - STM32_CAN lib
 - NMEA2000 lib


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

Inherited NMEA2000 object for STM32 MCU's with internal CAN
based setup. See also NMEA2000 library.
*/

#ifndef NMEA2000_STM32_H_
#define NMEA2000_STM32_H_

#include "NMEA2000.h"
#include "N2kMsg.h"
#include "STM32_CAN.hpp"

class N2kStream_STM32  : public N2kStream
{
	virtual size_t write(const uint8_t* data, size_t size) override;
	virtual int read() override;
	virtual int peek() override;
};

void     delay(uint32_t ms);
uint32_t millis(void);

//-----------------------------------------------------------------------------

class tNMEA2000_STM32 : public tSTM32_CAN, public tNMEA2000
{
  public:
  	tNMEA2000_STM32(CAN_HandleTypeDef *_canBus);
  	virtual ~tNMEA2000_STM32();

  	virtual void InitCANFrameBuffers() override;
    virtual bool CANOpen() override;
    virtual bool CANSendFrame(unsigned long id, unsigned char len, const unsigned char* buf, bool wait_sent = true) override;
    virtual bool CANGetFrame(unsigned long& id, unsigned char& len, unsigned char* buf) override;

  protected:
    N2kStream_STM32 Serial;

    // TODO NMEA2000.cpp uses Serial.print(...) for debugging
    // but if we are not using Arduino this Class is not defined
    // How can I create a N2kStream Serial instance which is available in the tNMEA2000 base class?
    // But I get an error: 'Serial' was not declared in this scope


};


#endif /* NMEA2000_STM32_H_ */

