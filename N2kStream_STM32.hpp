/*
 * N2kStream_STM32.hpp
 *
 *  Created on: Jan 11, 2023

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

#ifndef NMEA2000_STM32_N2KSTREAM_STM32_HPP_
#define NMEA2000_STM32_N2KSTREAM_STM32_HPP_

#include "N2kStream.h"

//-----------------------------------------------------------------------------
// derived class to use N2kStream with STM32
class N2kStream_STM32  : public N2kStream
{
  public:
	virtual size_t write(const uint8_t* data, size_t size) override;
	virtual int read() override;
	virtual int peek() override;
};



#endif /* NMEA2000_STM32_N2KSTREAM_STM32_HPP_ */
