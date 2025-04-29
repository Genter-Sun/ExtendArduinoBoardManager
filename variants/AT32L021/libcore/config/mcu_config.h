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
#ifndef __MCU_CONFIG_H
#define __MCU_CONFIG_H

/*=========================
   MCU configuration
 *=========================*/
/* System tick */
#define SYSTICK_TICK_FREQ                   1000 // Hz
#define SYSTICK_PRIORITY                    0

#define L021_series
#define IOMUX_NEW

/* Hardware Serial */
#define SERIAL_RX_BUFFER_SIZE               128
#define SERIAL_PREEMPTIONPRIORITY_DEFAULT   1
#define SERIAL_SUBPRIORITY_DEFAULT          3
#define SERIAL_CONFIG_DEFAULT               SERIAL_8N1

#define SERIAL_1_ENABLE                     1
#if SERIAL_1_ENABLE
#  define SERIAL_1_USART                    USART1
#  define SERIAL_1_USART_tx                 PA9
#  define SERIAL_1_USART_rx                 PA10
#  define SERIAL_1_IRQ                      USART1_IRQn
#  define SERIAL_1_CLOCK                    CRM_USART1_PERIPH_CLOCK
#  define SERIAL_1_IRQ_HANDLER_DEF()        void USART1_IRQHandler(void)
#endif

#define SERIAL_2_ENABLE                     1
#if SERIAL_2_ENABLE
#  define SERIAL_2_USART                    USART2
#  define SERIAL_2_USART_tx                 PA2
#  define SERIAL_2_USART_rx                 PA3
#  define SERIAL_2_IRQ                      USART2_IRQn
#  define SERIAL_2_CLOCK                    CRM_USART2_PERIPH_CLOCK
#  define SERIAL_2_IRQ_HANDLER_DEF()        void USART2_IRQHandler(void)
#endif

#define SERIAL_3_ENABLE                     1
#if SERIAL_3_ENABLE
#  define SERIAL_3_USART                    USART3
#  define SERIAL_3_USART_tx                 PB10
#  define SERIAL_3_USART_rx                 PB11
#  define SERIAL_3_IRQ                      USART4_3_IRQn
#  define SERIAL_3_CLOCK                    CRM_USART3_PERIPH_CLOCK
#  define SERIAL_3_IRQ_HANDLER_DEF()        void USART4_3_IRQHandler(void)
#endif

#define SERIAL_4_ENABLE                     1
#if SERIAL_4_ENABLE
#  define SERIAL_4_USART                    USART4
#  define SERIAL_4_USART_tx                 PA0
#  define SERIAL_4_USART_rx                 PA1
#  define SERIAL_4_IRQ                      USART4_3_IRQn
#  define SERIAL_4_CLOCK                    CRM_USART4_PERIPH_CLOCK
#  define SERIAL_4_IRQ_HANDLER_DEF()        void USART4_3_IRQHandler(void)
#endif

/* Wire (Software I2C) */
#define WIRE_USE_FULL_SPEED_I2C             0
#define WIRE_SDA_PIN                        PB7
#define WIRE_SCL_PIN                        PB6
#define WIRE_DELAY                          0
#define WIRE_BEGIN_TIMEOUT                  100 // ms
#define WIRE_BUFF_SIZE                      32

/* Hardware IIC */  
#define I2C_1_ADDRESS                       0xA0
#define I2C_1_CLKCTRL                       0x60F03333//100K   //0x20C02C4F 200K   //0x60F06C6C  50K  //0xF070F7F7  10K
#define I2C_1_SCL                           PB6
#define I2C_1_SDA                           PB7
#define I2C_1_DMA_TX_Channel                DMA1_CHANNEL2
#define I2C_1_DMA_TX_DMAMUX_Channel         FLEX_CHANNEL2
#define I2C_1_DMA_TX_DMAREQ                 DMA_FLEXIBLE_I2C1_TX
#define I2C_1_DMA_TX_IRQn                   DMA1_Channel3_2_IRQn
#define I2C_1_DMA_RX_Channel                DMA1_CHANNEL3
#define I2C_1_DMA_RX_DMAMUX_Channel         FLEX_CHANNEL3
#define I2C_1_DMA_RX_DMAREQ                 DMA_FLEXIBLE_I2C1_RX
#define I2C_1_DMA_RX_IRQn                   DMA1_Channel3_2_IRQn
#define I2C_1_IRQn                          I2C1_IRQn    

#define I2C_2_ADDRESS                       0xA0
#define I2C_2_CLKCTRL                       0x60F03333//100K   //0x20C02C4F 200K   //0x60F06C6C  50K  //0xF070F7F7  10K
#define I2C_2_SCL                           PA0
#define I2C_2_SDA                           PA1
#define I2C_2_DMA_TX_Channel                DMA1_CHANNEL4
#define I2C_2_DMA_TX_DMAMUX_Channel         FLEX_CHANNEL4
#define I2C_2_DMA_TX_DMAREQ                 DMA_FLEXIBLE_I2C2_TX
#define I2C_2_DMA_TX_IRQn                   DMA1_Channel5_4_IRQn
#define I2C_2_DMA_RX_Channel                DMA1_CHANNEL5
#define I2C_2_DMA_RX_DMAMUX_Channel         FLEX_CHANNEL5
#define I2C_2_DMA_RX_DMAREQ                 DMA_FLEXIBLE_I2C2_RX
#define I2C_2_DMA_RX_IRQn                   DMA1_Channel5_4_IRQn
#define I2C_2_IRQn                          I2C2_IRQn

/* SPI Class */
#define SPI_CLASS_AVR_COMPATIBILITY_MODE    1
#define SPI_CLASS_PIN_DEFINE_ENABLE         1

#define SPI_CLASS_1_ENABLE                  1
#if SPI_CLASS_1_ENABLE
#   define SPI_CLASS_1_SPI                  SPI1
#   define SPI_1_CLK                        PA5
#   define SPI_1_MOSI                       PA7
#   define SPI_1_MISO                       PA6
#endif

#define SPI_CLASS_2_ENABLE                  1
#if SPI_CLASS_2_ENABLE
#   define SPI_CLASS_2_SPI                  SPI2
#   define SPI_2_CLK                        PB13
#   define SPI_2_MOSI                       PB15
#   define SPI_2_MISO                       PB14
#endif

/* WString */
#define WSTRING_MEM_INCLUDE                 <stdlib.h>
#define WSTRING_MEM_REALLOC                 realloc
#define WSTRING_MEM_FREE                    free

/* Print */
#define PRINT_PRINTF_BUFFER_LENGTH          128

/* GPIO */
#define GPIO_DRIVE_DEFAULT                  GPIO_DRIVE_STRENGTH_STRONGER

/* External Interrupt  */
#define EXTI_PREEMPTIONPRIORITY_DEFAULT     2
#define EXTI_SUBPRIORITY_DEFAULT            1

/* Timer Interrupt */
#define TIMER_PREEMPTIONPRIORITY_DEFAULT    0
#define TIMER_SUBPRIORITY_DEFAULT           3

/* Tone */
#define TONE_TIMER_DEFAULT                  TIM1
#define TONE_PREEMPTIONPRIORITY_DEFAULT     0
#define TONE_SUBPRIORITY_DEFAULT            1

/* PWM */
#define PWM_RESOLUTION_DEFAULT              1000
#define PWM_FREQUENCY_DEFAULT               10000

#endif
