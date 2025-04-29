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
#include "gpio.h"

const Peripheral_pin_Typedef TMR_Pin_Map[TMR_PIN_MUX] = 
{   
    //tmr1
    {PA8,  GPIOA, TMR1, GPIO_MUX_1, GPIO_PINS_SOURCE8,  1},
    {PA9,  GPIOA, TMR1, GPIO_MUX_1, GPIO_PINS_SOURCE9,  2},
    {PA10, GPIOA, TMR1, GPIO_MUX_1, GPIO_PINS_SOURCE10, 3},
    {PA11, GPIOA, TMR1, GPIO_MUX_1, GPIO_PINS_SOURCE11, 4},
    //tmr3
    {PA6,  GPIOA, TMR3, GPIO_MUX_2, GPIO_PINS_SOURCE6, 1},
    {PA7,  GPIOA, TMR3, GPIO_MUX_2, GPIO_PINS_SOURCE7, 2},
    {PB0,  GPIOB, TMR3, GPIO_MUX_2, GPIO_PINS_SOURCE0, 3},
    {PB1,  GPIOB, TMR3, GPIO_MUX_2, GPIO_PINS_SOURCE1, 4},




};

const PinInfo_TypeDef PIN_MAP[PIN_MAX] =
{
    /*GPIO_Type* GPIOx;    //GPIOx address
      TIM_Type* TIMx;      //TIMx address
      ADC_Type* ADCx;      //ADCx address

      uint16_t GPIO_Pin_x;    //GPIO_Pin number
      uint8_t TimerChannel;   //timer channel
      uint8_t ADC_CHANNEL;    //ADC channel
    */
    {GPIOA, TIM1, ADC1,  GPIO_Pin_0, 0, ADC_CHANNEL_0}, /* PA0 */
    {GPIOA, TIM15, ADC1,  GPIO_Pin_1, 0, ADC_CHANNEL_1}, /* PA1 */
    {GPIOA, TIM15, ADC1,  GPIO_Pin_2, 1, ADC_CHANNEL_2}, /* PA2 */
    {GPIOA, TIM15, ADC1,  GPIO_Pin_3, 2, ADC_CHANNEL_3}, /* PA3 */
    {GPIOA, TIM14, ADC1,  GPIO_Pin_4, 1, ADC_CHANNEL_4}, /* PA4 */
    {GPIOA, NULL, ADC1,  GPIO_Pin_5, 0, ADC_CHANNEL_5}, /* PA5 */
    {GPIOA, TIM3, ADC1,  GPIO_Pin_6, 1, ADC_CHANNEL_6}, /* PA6 */
    {GPIOA, TIM3, ADC1,  GPIO_Pin_7, 2, ADC_CHANNEL_7}, /* PA7 */
    {GPIOA, TIM1, NULL,  GPIO_Pin_8, 1, ADC_CHANNEL_X}, /* PA8 */
    {GPIOA, TIM1, NULL,  GPIO_Pin_9, 2, ADC_CHANNEL_X}, /* PA9 */
    {GPIOA, TIM1, NULL, GPIO_Pin_10, 3, ADC_CHANNEL_X}, /* PA10 */
    {GPIOA, TIM1, NULL, GPIO_Pin_11, 4, ADC_CHANNEL_X}, /* PA11 */
    {GPIOA, NULL, NULL, GPIO_Pin_12, 0, ADC_CHANNEL_X}, /* PA12 */
    {GPIOA, NULL, NULL, GPIO_Pin_13, 0, ADC_CHANNEL_X}, /* PA13 */
    {GPIOA, NULL, NULL, GPIO_Pin_14, 0, ADC_CHANNEL_X}, /* PA14 */
    {GPIOA, NULL, NULL, GPIO_Pin_15, 0, ADC_CHANNEL_X}, /* PA15 */

    {GPIOB, TIM3, ADC1,  GPIO_Pin_0, 3, ADC_CHANNEL_8}, /* PB0 */
    {GPIOB, TIM3, ADC1,  GPIO_Pin_1, 4, ADC_CHANNEL_9}, /* PB1 */
    {GPIOB, NULL, ADC1,  GPIO_Pin_2, 0, ADC_CHANNEL_10}, /* PB2 */
    {GPIOB, NULL, NULL,  GPIO_Pin_3, 2, ADC_CHANNEL_X}, /* PB3 */
    {GPIOB, TIM3, NULL,  GPIO_Pin_4, 1, ADC_CHANNEL_X}, /* PB4 */
    {GPIOB, TIM3, NULL,  GPIO_Pin_5, 2, ADC_CHANNEL_X}, /* PB5 */
    {GPIOB, NULL, NULL,  GPIO_Pin_6, 0, ADC_CHANNEL_X}, /* PB6 */
    {GPIOB, NULL, NULL,  GPIO_Pin_7, 0, ADC_CHANNEL_X}, /* PB7 */
    {GPIOB, TIM16, NULL,  GPIO_Pin_8, 1, ADC_CHANNEL_X}, /* PB8 */
    {GPIOB, TIM17, NULL,  GPIO_Pin_9, 1, ADC_CHANNEL_X}, /* PB9 */
    {GPIOB, NULL, NULL, GPIO_Pin_10, 0, ADC_CHANNEL_X}, /* PB10 */
    {GPIOB, NULL, NULL, GPIO_Pin_11, 0, ADC_CHANNEL_X}, /* PB11 */
    {GPIOB, NULL, ADC1, GPIO_Pin_12, 0, ADC_CHANNEL_11}, /* PB12 */
    {GPIOB, NULL, ADC1, GPIO_Pin_13, 0, ADC_CHANNEL_12}, /* PB13 */
    {GPIOB, TIM15, ADC1, GPIO_Pin_14, 1, ADC_CHANNEL_13},/* PB14 */
    {GPIOB, TIM15, ADC1, GPIO_Pin_15, 2, ADC_CHANNEL_14}, /* PB15 */

    {GPIOC, NULL, NULL, GPIO_Pin_13, 0, ADC_CHANNEL_X}, /* PC13 */
    {GPIOC, NULL, NULL, GPIO_Pin_14, 0, ADC_CHANNEL_X}, /* PC14 */
    {GPIOC, NULL, NULL, GPIO_Pin_15, 0, ADC_CHANNEL_X}, /* PC15 */

    {GPIOF, TIM1, NULL,  GPIO_Pin_0, 1, ADC_CHANNEL_X}, /* PF0 */
    {GPIOF, NULL, NULL,  GPIO_Pin_1, 0, ADC_CHANNEL_X}, /* PF1 */
    {GPIOF, NULL, NULL,  GPIO_Pin_6, 0, ADC_CHANNEL_X}, /* PF6 */
    {GPIOF, NULL, NULL,  GPIO_Pin_7, 0, ADC_CHANNEL_X}, /* PF7 */


};

/**
  * @brief  GPIO init
  * @param  GPIOx: GPIO address
  * @param  GPIO_Pin_x: GPIO number
  * @param  GPIO_Mode_x: GPIO mode
  * @param  GPIO_Speed_x: GPIO speed
  * @retval none
  */
void GPIOx_Init(
    gpio_type* GPIOx,
    uint16_t GPIO_Pin_x,
    PinMode_TypeDef Mode,
    gpio_drive_type GPIO_Drive_x
)
{
    gpio_init_type gpio_init_struct;
    crm_periph_clock_type CRM_GPIOx_PERIPH_CLOCK;

    if(GPIOx == GPIOA)     
        CRM_GPIOx_PERIPH_CLOCK = CRM_GPIOA_PERIPH_CLOCK;
    else if(GPIOx == GPIOB)
        CRM_GPIOx_PERIPH_CLOCK = CRM_GPIOB_PERIPH_CLOCK;
    else if(GPIOx == GPIOC)
        CRM_GPIOx_PERIPH_CLOCK = CRM_GPIOC_PERIPH_CLOCK;
    #ifdef GPIOF
    else if(GPIOx == GPIOF)
        CRM_GPIOx_PERIPH_CLOCK = CRM_GPIOF_PERIPH_CLOCK;
    #endif /*GPIOF*/
    else return;
	
	gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_pins = GPIO_Pin_x;
    gpio_init_struct.gpio_drive_strength = GPIO_Drive_x;
    
    if(Mode == INPUT)
    {
        gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    }
    else if(Mode == INPUT_PULLUP)
    {
        gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
        gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    }
    else if(Mode == INPUT_PULLDOWN)
    {
        gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
        gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;
    }
    else if(Mode == INPUT_ANALOG)
    {
        gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    }
    else if(Mode == OUTPUT)
    {
        gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    }
    else if(Mode == OUTPUT_OPEN_DRAIN)
    {
        gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
    }
    else if(Mode == OUTPUT_AF_PP)
    {
        gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    }
    else if(Mode == OUTPUT_AF_OD)
    {
        gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
    }
    else
    {
        return;
    }

    crm_periph_clock_enable(CRM_GPIOx_PERIPH_CLOCK, TRUE);
    gpio_init(GPIOx, &gpio_init_struct);
}

/**
  * @brief  disable JTAG pin
  * @param  none
  * @retval none
  */
void GPIO_JTAG_Disable(void)
{
//    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_AFIO, ENABLE);
//    GPIO_PinsRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
}

/**
  * @brief  get GPIOx number
  * @param  Pin: pin number
  * @retval none
  */
uint8_t GPIO_GetPortNum(uint8_t Pin)
{
    uint8_t retval = 0xFF;
    uint8_t index;
    gpio_type* GPIOx = PIN_MAP[Pin].GPIOx;

    gpio_type* GPIO_Map[] =
    {
        GPIOA,
        GPIOB,
        GPIOC,
        #ifdef GPIOD
        GPIOD,
        #endif
        #ifdef GPIOE
        GPIOE,
        #endif       
        #ifdef GPIOF
        GPIOF,
        #endif
    };

    for(index = 0; index < sizeof(GPIO_Map) / sizeof(GPIO_Map[0]); index++)
    {
        if(GPIOx == GPIO_Map[index])
        {
            retval = index;
            break;
        }
    }

    return retval;
}

/**
  * @brief  get Pin Source
  * @param  GPIO_Pin_x: GPIO number
  * @retval none
  */
uint8_t GPIO_GetPinSource(uint16_t GPIO_Pin_x)
{
    uint8_t PinSource = 0;
    while(GPIO_Pin_x > 1)
    {
        GPIO_Pin_x >>= 1;
        PinSource++;
    }
    return PinSource;
}

/**
  * @brief  get pin number
  * @param  Pin: pin number
  * @retval none
  */
uint8_t GPIO_GetPinNum(uint8_t Pin)
{
    return GPIO_GetPinSource(PIN_MAP[Pin].GPIO_Pin_x);
}


