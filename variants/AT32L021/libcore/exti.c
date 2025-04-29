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
#include "exti.h"
#include "gpio.h"

#define EXTI_GetPortSourceGPIOx(Pin) GPIO_GetPortNum(Pin)
#define EXTI_GetPinSourcex(Pin)      GPIO_GetPinNum(Pin)

static EXTI_CallbackFunction_t EXTI_Function[16] = {0};

/**
  * @brief  get external interrupt IRQ channel
  * @param  Pin: pin number
  * @retval interrupt IRQ channel number
  */
static IRQn_Type EXTI_GetIRQn(uint8_t Pin)
{
    IRQn_Type EXINTx_IRQn = EXINT1_0_IRQn;
    uint8_t Pinx = GPIO_GetPinNum(Pin);

    if(Pinx <= 1)
    {
        EXINTx_IRQn = EXINT1_0_IRQn;
    }
    else if(Pinx >= 2 && Pinx <= 3)
    {
        EXINTx_IRQn = EXINT3_2_IRQn;
    }
    else if(Pinx >= 4 && Pinx <= 15)
    {
        EXINTx_IRQn = EXINT15_4_IRQn;
    }

    return EXINTx_IRQn;
}

/**
  * @brief  external interrupt init
  * @param  Pin: pin number
  * @param  Function: callback function
  * @param  line_polarity: trigger polarity
  * @param  PreemptionPriority: Preemption Priority
  * @param  SubPriority: Sub Priority
  * @retval none
  */
void EXTIx_Init(
    uint8_t Pin,
    EXTI_CallbackFunction_t Function,
    exint_polarity_config_type line_polarity,
    uint8_t PreemptionPriority,
    uint8_t SubPriority
)
{
    gpio_init_type gpio_init_struct;
	exint_init_type exint_init_struct;
    uint8_t Pinx;

    if(!IS_PIN(Pin))
        return;
    
    if(PIN_MAP[Pin].GPIOx == GPIOA)
        crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    else if(PIN_MAP[Pin].GPIOx == GPIOB)
        crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    else if(PIN_MAP[Pin].GPIOx == GPIOC)
        crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    else if(PIN_MAP[Pin].GPIOx == GPIOF)
        crm_periph_clock_enable(CRM_GPIOF_PERIPH_CLOCK, TRUE);

    Pinx = GPIO_GetPinNum(Pin);

    if(Pinx > 15)
        return;

    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_pins = 1 << Pinx;
    gpio_init(PIN_MAP[Pin].GPIOx, &gpio_init_struct);

    EXTI_Function[Pinx] = Function;

	crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
	scfg_exint_line_config(GPIO_GetPortNum(Pin), (gpio_pins_source_type)Pinx);
    exint_default_para_init(&exint_init_struct);
    exint_init_struct.line_select = 1 << Pinx;
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPUT;
    exint_init_struct.line_polarity = line_polarity;
    exint_init_struct.line_enable = TRUE;
    exint_init(&exint_init_struct);

    nvic_irq_enable(EXTI_GetIRQn(Pin), PreemptionPriority, SubPriority);

}

/**
  * @brief  enable external interrupt (Arduino)
  * @param  Pin: pin number
  * @param  function: callback function
  * @param  line_polarity: trigger polarity
  * @retval none
  */
void attachInterrupt(uint8_t Pin, EXTI_CallbackFunction_t Function, exint_polarity_config_type line_polarity)
{
    EXTIx_Init(
        Pin,
        Function,
        line_polarity,
        EXTI_PREEMPTIONPRIORITY_DEFAULT,
        EXTI_SUBPRIORITY_DEFAULT
    );
}

/**
  * @brief  disable external interrupt (Arduino)
  * @param  Pin: pin number
  * @retval none
  */
void detachInterrupt(uint8_t Pin)
{
    if(!IS_PIN(Pin))
        return;

    nvic_irq_disable(EXTI_GetIRQn(Pin));
}

#define EXTIx_IRQHANDLER(n) \
do{\
    if(exint_flag_get(EXINT_LINE_##n) != RESET)\
    {\
        if(EXTI_Function[n]) EXTI_Function[n]();\
        exint_flag_clear(EXINT_LINE_##n);\
    }\
}while(0)

/**
  * @brief  external interrupt entry 0/1
  * @param  none
  * @retval none
  */
void EXINT1_0_IRQHandler(void)
{
    EXTIx_IRQHANDLER(0);
    EXTIx_IRQHANDLER(1);
}

/**
  * @brief  external interrupt entry 2/3
  * @param  none
  * @retval none
  */
void EXINT3_2_IRQHandler(void)
{
    EXTIx_IRQHANDLER(2);
    EXTIx_IRQHANDLER(3);
}

/**
  * @brief  external interrupt entry 4~15
  * @param  none
  * @retval none
  */
void EXINT15_4_IRQHandler(void)
{
    EXTIx_IRQHANDLER(4);
    EXTIx_IRQHANDLER(5);
    EXTIx_IRQHANDLER(6);
    EXTIx_IRQHANDLER(7);
    EXTIx_IRQHANDLER(8);
    EXTIx_IRQHANDLER(9);
    EXTIx_IRQHANDLER(10);
    EXTIx_IRQHANDLER(11);
    EXTIx_IRQHANDLER(12);
    EXTIx_IRQHANDLER(13);
    EXTIx_IRQHANDLER(14);
    EXTIx_IRQHANDLER(15);
}

