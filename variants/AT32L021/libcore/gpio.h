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
#ifndef __GPIO_H
#define __GPIO_H

#include "mcu_type.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef NULL
#  define NULL ((void*)0)
#endif

#define ADC_CHANNEL_X ((uint8_t)0xFF)

#define IS_PIN(Pin)     (Pin < PIN_MAX)
#define IS_ADC_PIN(Pin) (IS_PIN(Pin) && PIN_MAP[Pin].ADCx != NULL && PIN_MAP[Pin].ADC_Channel  != ADC_CHANNEL_X)
#define IS_PWM_PIN(Pin) (IS_PIN(Pin) && PIN_MAP[Pin].TIMx != NULL && PIN_MAP[Pin].TimerChannel != 0)

#define GPIO_HIGH(GPIOX,GPIO_PIN_X)    ((GPIOX)->scr    = (GPIO_PIN_X))
#define GPIO_LOW(GPIOX,GPIO_PIN_X)     ((GPIOX)->clr     = (GPIO_PIN_X))
#define GPIO_READ(GPIOX,GPIO_PIN_X)   (((GPIOX)->idt   & (GPIO_PIN_X))!=0)
#define GPIO_TOGGLE(GPIOX,GPIO_PIN_X)  ((GPIOX)->odt  ^= (GPIO_PIN_X))

#define portInputRegister(Port)     (&(Port->idt))
#define portOutputRegister(Port)    (&(Port->odt))

#define analogInPinToBit(Pin)       (Pin)
#define digitalPinToInterrupt(Pin)  (Pin)
#define digitalPinToPort(Pin)       (PIN_MAP[Pin].GPIOx)
#define digitalPinToBitMask(Pin)    (PIN_MAP[Pin].GPIO_Pin_x)

#define digitalWrite_HIGH(Pin)      GPIO_HIGH  (PIN_MAP[Pin].GPIOx, PIN_MAP[Pin].GPIO_Pin_x)
#define digitalWrite_LOW(Pin)       GPIO_LOW   (PIN_MAP[Pin].GPIOx, PIN_MAP[Pin].GPIO_Pin_x)
#define digitalRead_FAST(Pin)       GPIO_READ  (PIN_MAP[Pin].GPIOx, PIN_MAP[Pin].GPIO_Pin_x)
#define togglePin(Pin)              GPIO_TOGGLE(PIN_MAP[Pin].GPIOx, PIN_MAP[Pin].GPIO_Pin_x)

#define LED_BUILTIN PF6

#ifndef PA0
    #define PA0  0
#endif
#ifndef PA1
    #define PA1  1
#endif
#ifndef PA2
    #define PA2  2
#endif
#ifndef PA3
    #define PA3  3
#endif
#ifndef PA4
    #define PA4  4
#endif
#ifndef PA5
    #define PA5  5
#endif
#ifndef PA6
    #define PA6  6
#endif
#ifndef PA7
    #define PA7  7
#endif
#ifndef PA8
    #define PA8  8
#endif
#ifndef PA9
    #define PA9  9
#endif
#ifndef PA10
    #define PA10  10
#endif
#ifndef PA11
    #define PA11  11
#endif
#ifndef PA12
    #define PA12  12
#endif
#ifndef PA13
    #define PA13  13
#endif
#ifndef PA14
    #define PA14  14
#endif
#ifndef PA15
    #define PA15  15
#endif
#ifndef PB0
    #define PB0  16
#endif
#ifndef PB1
    #define PB1  17
#endif
#ifndef PB2
    #define PB2  18
#endif
#ifndef PB3
    #define PB3  19
#endif
#ifndef PB4
    #define PB4  20
#endif
#ifndef PB5
    #define PB5  21
#endif
#ifndef PB6
    #define PB6  22
#endif
#ifndef PB7
    #define PB7  23
#endif
#ifndef PB8
    #define PB8  24
#endif
#ifndef PB9
    #define PB9  25
#endif
#ifndef PB10
    #define PB10  26
#endif
#ifndef PB11
    #define PB11  27
#endif
#ifndef PB12
    #define PB12  28
#endif
#ifndef PB13
    #define PB13  29
#endif
#ifndef PB14
    #define PB14  30
#endif
#ifndef PB15
    #define PB15  31
#endif
#ifndef PC13
    #define PC13  32
#endif
#ifndef PC14
    #define PC14  33
#endif
#ifndef PC15
    #define PC15  34
#endif
#ifndef PF0
    #define PF0   35
#endif
#ifndef PF1
    #define PF1   36
#endif
#ifndef PF6
    #define PF6   37
#endif
#ifndef PF7
    #define PF7   38
#endif
#ifndef PIN_MAX
    #define PIN_MAX 39
#endif

#define Pin_TypeDef uint8_t 

typedef struct
{
    gpio_type* GPIOx;
    tmr_type* TIMx;
    adc_type* ADCx;
    uint16_t GPIO_Pin_x;
    uint8_t TimerChannel;
    uint8_t ADC_Channel;
} PinInfo_TypeDef;

typedef struct
{
    Pin_TypeDef PINx;
    gpio_type* GPIOx;
    void* Peripheralx;
    gpio_mux_sel_type  mux_number;
    gpio_pins_source_type pins_source;
    uint8_t tmr_channel;

} Peripheral_pin_Typedef;


#define TMR_PIN_MUX  12
extern const Peripheral_pin_Typedef TMR_Pin_Map[TMR_PIN_MUX];

typedef enum
{
    INPUT,
    INPUT_PULLUP,
    INPUT_PULLDOWN,
    INPUT_ANALOG,
    INPUT_ANALOG_DMA,
    OUTPUT,
    OUTPUT_OPEN_DRAIN,
    OUTPUT_AF_OD,
    OUTPUT_AF_PP,
    PWM
} PinMode_TypeDef;

extern const PinInfo_TypeDef PIN_MAP[PIN_MAX];

void GPIOx_Init(
    gpio_type* GPIOx,
    uint16_t GPIO_Pin_x,
    PinMode_TypeDef Mode,
    gpio_drive_type GPIO_Drive_x
);
void GPIO_JTAG_Disable(void);
uint8_t GPIO_GetPortNum(uint8_t Pin);
uint8_t GPIO_GetPinNum(uint8_t Pin);
uint8_t GPIO_GetPinSource(uint16_t GPIO_Pin_x);

#ifdef __cplusplus
}// extern "C"
#endif

#endif
