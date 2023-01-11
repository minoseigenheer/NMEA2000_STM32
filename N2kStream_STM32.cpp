/*
 * N2kStream_STM32.cpp
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


#include "N2kStream_STM32.hpp"
#include "NMEA2000_STM32.hpp"


// N2kStream is used for debugging
size_t N2kStream_STM32::write(const uint8_t* data, size_t size) {
	return _write(0, reinterpret_cast<char*>(const_cast<uint8_t*>(data)), (int)size);
}
// TODO implement STM32 read
int N2kStream_STM32::read() {
	return -1;
}
// TODO implement STM32 peek
int N2kStream_STM32::peek() {
	return -1;
}

