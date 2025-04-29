/*
 * MIT License
 * Copyright (c) 2017 - 2022 _VIFEXTech
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef _PERIPHERALPINS_H
#define _PERIPHERALPINS_H

#include "pinmap.h"

//*** PWM ***
extern const PinMap PinMap_PWM[];

//*** ADC ***
extern const PinMap PinMap_ADC[];

//*** SPI ***
extern const PinMap PinMap_SPI_CS[];
extern const PinMap PinMap_SPI_SCK[];
extern const PinMap PinMap_SPI_MISO[];
extern const PinMap PinMap_SPI_MOSI[];

//*** USART ***
extern const PinMap PinMap_USART_RX[];
extern const PinMap PinMap_USART_TX[];
extern const PinMap PinMap_USART_RTS[];
extern const PinMap PinMap_USART_CTS[];

//*** IIC ***
extern const PinMap PinMap_I2C_SCL[];
extern const PinMap PinMap_I2C_SDA[];

#endif

