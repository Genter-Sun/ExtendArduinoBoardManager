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
#include "Arduino.h"
#include "pinmap.h"
#include "gpio.h"
/**
  * @brief  判断当前pin脚所属外设
  * @param  Pin: 引脚编号
  * @param  map: 外设引脚定义表
  * @retval 外设
  */
void *pinmap_find_peripheral(uint8_t pin, const PinMap *map)
{
  while (map->pin != NC) {
    if (map->pin == pin) {
      return map->peripheral;
    }
    map++;
  }
  return NP;
}

/**
  * @brief  判断当前pin脚是否支持PWM
  * @param  Pin: 引脚编号
  * @param  TMR_Pin_Map: timer引脚定义表
  * @retval 是否支持
  */
bool GPIO_isPWMpinSource(uint8_t pin, const PinMap *PinMap_PWM)
{
  if (pin != NC) {
    while (PinMap_PWM->pin != NC) {
      if (PinMap_PWM->pin == pin) {
        return true;
      }
      PinMap_PWM++;
    }
  }
  return false;
}

/**
  * @brief  判断当前pin脚所属外设
  * @param  Pin: 引脚编号
  * @param  map: 外设引脚定义表
  * @retval 外设
  */
void *pinmap_peripheral(uint8_t pin, const PinMap *map)
{
  void *peripheral = NP;

  if (pin != NC) {
    peripheral = pinmap_find_peripheral(pin, map);
  }
  return peripheral;
}

uint32_t pinmap_find_function(uint8_t pin, const PinMap *map)
{
  while (map->pin != NC) {
    if (map->pin == pin) {
      return map->function;
    }
    map++;
  }
  return (uint32_t)NC;
}

/**
  * @brief  获取当前pin脚所属外设功能
  * @param  Pin: 引脚编号
  * @param  map: 外设引脚定义表
  * @retval 功能
  */
uint32_t pinmap_function(uint8_t pin, const PinMap *map)
{
  uint32_t function = (uint32_t)NC;

  if (pin != NC) {
    function = pinmap_find_function(pin, map);
  }
  return function;   
}

void pinmap_pinout(uint8_t pin, const PinMap *map)
{
  if (pin == NC) {
    return;
  }

  while (map->pin != NC) {
    if (map->pin == pin) {
      /* Get the pin informations */
      crm_periph_clock_type CRM_GPIOx_PERIPH_CLOCK = 0;
      gpio_init_type  gpio_init_struct;
      uint32_t mode  = AT_PIN_FUNCTION(map->function);
      gpio_type* port = map->GPIOx;
      uint32_t ll_pin  = (gpio_pins_source_type)(map->pins_source);
      uint32_t ll_mode = 0;

      if(port == GPIOA) CRM_GPIOx_PERIPH_CLOCK = CRM_GPIOA_PERIPH_CLOCK;
      else if(port == GPIOB) CRM_GPIOx_PERIPH_CLOCK = CRM_GPIOB_PERIPH_CLOCK;
      else if(port == GPIOC) CRM_GPIOx_PERIPH_CLOCK = CRM_GPIOC_PERIPH_CLOCK;
#ifdef GPIOF
      else if(port == GPIOF)CRM_GPIOx_PERIPH_CLOCK = CRM_GPIOF_PERIPH_CLOCK;
#endif /*GPIOF*/

      crm_periph_clock_enable(CRM_GPIOx_PERIPH_CLOCK, TRUE);

      switch (mode) {
        case AT_PIN_INPUT:
          ll_mode = GPIO_MODE_INPUT;
          break;
        case AT_PIN_OUTPUT:
          ll_mode = GPIO_MODE_OUTPUT;
          break;
        case AT_PIN_ALTERNATE:
          ll_mode = GPIO_MODE_MUX;
          break;
        case AT_PIN_ANALOG:
          ll_mode = GPIO_MODE_ANALOG;
          break;
        default:
          break;
      }
      gpio_default_para_init(&gpio_init_struct);
      switch ((gpio_pins_source_type)(map->pins_source)) {
        case GPIO_PINS_SOURCE0:
          gpio_init_struct.gpio_pins = GPIO_PINS_0;
          break;
        case GPIO_PINS_SOURCE1:
          gpio_init_struct.gpio_pins = GPIO_PINS_1;
          break;
        case GPIO_PINS_SOURCE2:
          gpio_init_struct.gpio_pins = GPIO_PINS_2;
          break;
        case GPIO_PINS_SOURCE3:
          gpio_init_struct.gpio_pins = GPIO_PINS_3;
          break;
        case GPIO_PINS_SOURCE4:
          gpio_init_struct.gpio_pins = GPIO_PINS_4;
          break;
        case GPIO_PINS_SOURCE5:
          gpio_init_struct.gpio_pins = GPIO_PINS_5;
          break;
        case GPIO_PINS_SOURCE6:
          gpio_init_struct.gpio_pins = GPIO_PINS_6;
          break;
        case GPIO_PINS_SOURCE7:
          gpio_init_struct.gpio_pins = GPIO_PINS_7;
          break;
        case GPIO_PINS_SOURCE8:
          gpio_init_struct.gpio_pins = GPIO_PINS_8;
          break;
        case GPIO_PINS_SOURCE9:
          gpio_init_struct.gpio_pins = GPIO_PINS_9;
          break;
        case GPIO_PINS_SOURCE10:
          gpio_init_struct.gpio_pins = GPIO_PINS_10;
          break;
        case GPIO_PINS_SOURCE11:
          gpio_init_struct.gpio_pins = GPIO_PINS_11;
          break;
        case GPIO_PINS_SOURCE12:
          gpio_init_struct.gpio_pins = GPIO_PINS_12;
          break;
        case GPIO_PINS_SOURCE13:
          gpio_init_struct.gpio_pins = GPIO_PINS_13;
          break;
        case GPIO_PINS_SOURCE14:
          gpio_init_struct.gpio_pins = GPIO_PINS_14;
          break;
        case GPIO_PINS_SOURCE15:
          gpio_init_struct.gpio_pins = GPIO_PINS_15;
          break;
        default:
          break;
      }
      
      gpio_init_struct.gpio_mode = ll_mode;
      gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
      if ((mode == AT_PIN_OUTPUT) || (mode == AT_PIN_ALTERNATE)) 
      {
        if (AT_PIN_OD(map->function)) {
          gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
        } else {
          gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
        }
      }
      gpio_init_struct.gpio_pull = (gpio_output_type)AT_PIN_PUPD(map->function);
      gpio_init(port, &gpio_init_struct);

      gpio_pin_mux_config(port, ll_pin, (gpio_mux_sel_type )(AT_PIN_AFNUM(map->function)));
      return;
    }
    map++;
  }
  while(1);
}

void peripheral_pin_config(uint8_t pin, const PinMap *map)
{
  if (pin == NC) {
    return;
  }

  while (map->pin != NC)
  {
    if (map->pin == pin) {
      pinMode(map->pin, OUTPUT_AF_PP);
      gpio_pin_mux_config(map->GPIOx, map->pins_source, (gpio_mux_sel_type )(AT_PIN_AFNUM(pinmap_function(pin, map))));
      return;
    }
    map ++;
  }
}




