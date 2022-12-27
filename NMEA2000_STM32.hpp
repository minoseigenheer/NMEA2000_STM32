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


class tNMEA2000_STM32 : public tSTM32_CAN, public tNMEA2000
{
  public:
  	tNMEA2000_STM32(CAN_HandleTypeDef *_canBus, tSTM32_CAN::CANbaudRatePrescaler _CANbaudRate);
  	virtual ~tNMEA2000_STM32();

    virtual bool CANOpen() override;
    virtual bool CANSendFrame(unsigned long id, unsigned char len, const unsigned char* buf, bool wait_sent = true) override;
    virtual bool CANGetFrame(unsigned long& id, unsigned char& len, unsigned char* buf) override;


};

//-----------------------------------------------------------------------------

void     delay(uint32_t ms);
uint32_t millis(void);



#endif /* NMEA2000_STM32_H_ */

