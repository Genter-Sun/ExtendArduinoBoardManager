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
#ifndef __PINMAP_H
#define __PINMAP_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "at32l021_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  AT_PIN_INPUT = 0,
  AT_PIN_OUTPUT = 1,
  AT_PIN_ALTERNATE = 2,
  AT_PIN_ANALOG = 3,
} ATPinFunction;

#define AT_PIN_FUNCTION_MASK 0x07
#define AT_PIN_FUNCTION_SHIFT 0
#define AT_PIN_FUNCTION_BITS (AT_PIN_FUNCTION_MASK << AT_PIN_FUNCTION_SHIFT)

#define AT_PIN_OD_MASK 0x01
#define AT_PIN_OD_SHIFT 3
#define AT_PIN_OD_BITS (AT_PIN_OD_MASK << AT_PIN_OD_SHIFT)

#define AT_PIN_PUPD_MASK 0x03
#define AT_PIN_PUPD_SHIFT 4
#define AT_PIN_PUPD_BITS (AT_PIN_PUPD_MASK << AT_PIN_PUPD_SHIFT)

#define AT_PIN_SPEED_MASK 0x03
#define AT_PIN_SPEED_SHIFT 6
#define AT_PIN_SPEED_BITS (AT_PIN_SPEED_MASK << AT_PIN_SPEED_SHIFT)

#define AT_PIN_AFNUM_MASK 0x7F
#define AT_PIN_AFNUM_SHIFT 8
#define AT_PIN_AFNUM_BITS (AT_PIN_AFNUM_MASK << AT_PIN_AFNUM_SHIFT)

#define AT_PIN_CHAN_MASK 0x1F
#define AT_PIN_CHAN_SHIFT 15
#define AT_PIN_CHANNEL_BIT (AT_PIN_CHAN_MASK << AT_PIN_CHAN_SHIFT)

#define AT_PIN_INV_MASK 0x01
#define AT_PIN_INV_SHIFT 20
#define AT_PIN_INV_BIT (AT_PIN_INV_MASK << AT_PIN_INV_SHIFT)

#define AT_PIN_AN_CTRL_MASK 0x01
#define AT_PIN_AN_CTRL_SHIFT 21
#define AT_PIN_ANALOG_CONTROL_BIT (AT_PIN_AN_CTRL_MASK << AT_PIN_AN_CTRL_SHIFT)

#define AT_PIN_FUNCTION(X)         (((X) >> AT_PIN_FUNCTION_SHIFT) & AT_PIN_FUNCTION_MASK)
#define AT_PIN_OD(X)               (((X) >> AT_PIN_OD_SHIFT) & AT_PIN_OD_MASK)
#define AT_PIN_PUPD(X)             (((X) >> AT_PIN_PUPD_SHIFT) & AT_PIN_PUPD_MASK)
#define AT_PIN_SPEED(X)            (((X) >> AT_PIN_SPEED_SHIFT) & AT_PIN_SPEED_MASK)
#define AT_PIN_AFNUM(X)            (((X) >> AT_PIN_AFNUM_SHIFT) & AT_PIN_AFNUM_MASK)
#define AT_PIN_CHANNEL(X)          (((X) >> AT_PIN_CHAN_SHIFT) & AT_PIN_CHAN_MASK)
#define AT_PIN_INVERTED(X)         (((X) >> AT_PIN_INV_SHIFT) & AT_PIN_INV_MASK)
#define AT_PIN_ANALOG_CONTROL(X)   (((X) >> AT_PIN_AN_CTRL_SHIFT) & AT_PIN_AN_CTRL_MASK)
#define AT_PIN_MODE(X)             ((AT_PIN_OD((X)) << 4) | \
                                      (AT_PIN_FUNCTION((X)) & (~AT_PIN_OD_BITS)))

#define AT_MODE_INPUT               (AT_PIN_INPUT)
#define AT_MODE_OUTPUT_PP           (AT_PIN_OUTPUT)
#define AT_MODE_OUTPUT_OD           (AT_PIN_OUTPUT | AT_PIN_OD_BITS)
#define AT_MODE_AF_PP               (AT_PIN_ALTERNATE)
#define AT_MODE_AF_OD               (AT_PIN_ALTERNATE | AT_PIN_OD_BITS)
#define AT_MODE_ANALOG              (AT_PIN_ANALOG)
#define AT_MODE_ANALOG_ADC_CONTROL  (AT_PIN_ANALOG | AT_PIN_ANALOG_CONTROL_BIT)

#define AT_PIN_DEFINE_EXT(FUNC_OD, PUPD, AFNUM, CHAN, INV) \
                                            ((int)(FUNC_OD) |\
                       ((PUPD   & AT_PIN_PUPD_MASK) << AT_PIN_PUPD_SHIFT) |\
                       ((AFNUM  & AT_PIN_AFNUM_MASK) << AT_PIN_AFNUM_SHIFT) |\
                       ((CHAN   & AT_PIN_CHAN_MASK) << AT_PIN_CHAN_SHIFT) |\
                       ((INV    & AT_PIN_INV_MASK) << AT_PIN_INV_SHIFT))

#define AT_PIN_DEFINE(FUNC_OD, PUPD, AFNUM)  ((int)(FUNC_OD) |\
                          ((PUPD  & AT_PIN_PUPD_MASK) << AT_PIN_PUPD_SHIFT) |\
                          ((AFNUM & AT_PIN_AFNUM_MASK) << AT_PIN_AFNUM_SHIFT))

#define AT_PIN_DATA_EXT(FUNC_OD, PUPD, AFNUM, CHANNEL, INVERTED) \
            AT_PIN_DEFINE_EXT(FUNC_OD, PUPD, AFNUM, CHANNEL, INVERTED)



#define AT_PIN_DATA(FUNC_OD, PUPD, AFNUM) \
            AT_PIN_DEFINE(FUNC_OD, PUPD, AFNUM)


// No pin
#define NC       0xFFU
// No peripheral
#define NP       0U

#ifndef PWM_FREQUENCY
  #define PWM_FREQUENCY               1000
#endif
#ifndef PWM_MAX_DUTY_CYCLE
  #define PWM_MAX_DUTY_CYCLE          4095
#endif

typedef struct {
  uint8_t pin;
  gpio_type* GPIOx;
  void *peripheral;
  int function;
  gpio_pins_source_type pins_source;
} PinMap;

void *pinmap_find_peripheral(uint8_t pin, const PinMap *map);
bool GPIO_isPWMpinSource(uint8_t Pin, const PinMap* PinMap_PWM);
void *pinmap_peripheral(uint8_t Pin, const PinMap* map);
uint32_t pinmap_find_function(uint8_t pin, const PinMap *map);
uint32_t pinmap_function(uint8_t pin, const PinMap *map);
void pinmap_pinout(uint8_t pin, const PinMap *map);
void peripheral_pin_config(uint8_t pin, const PinMap *map);

#ifdef __cplusplus
}
#endif


#endif