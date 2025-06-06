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
#include "HardwareSerial.h"

#ifdef IOMUX_NEW
#include "pinmap.h"
#include "PeripheralPins.h"
#endif



#define SERIAL_GET_WORDLENGTH(SERIAL_x)    ((uint16_t)(SERIAL_x&0xF000))
#define SERIAL_GET_PARITY(SERIAL_x)        ((uint16_t)(SERIAL_x&0x0F00))
#define SERIAL_GET_STOPBITS(SERIAL_x)      ((uint16_t)((SERIAL_x&0x00F0) << 8))
typedef struct
{
    usart_data_bit_num_type data_bit;
    usart_parity_selection_type parity_selection;
    usart_stop_bit_num_type stop_bit;
} SERIAL_ConfigGrp_t;

static const SERIAL_ConfigGrp_t SERIAL_ConfigGrp[] =
{
    {USART_DATA_8BITS, USART_PARITY_EVEN, USART_STOP_1_BIT},   // SERIAL_7E1
    {USART_DATA_8BITS, USART_PARITY_EVEN, USART_STOP_2_BIT},   // SERIAL_7E2
    {USART_DATA_8BITS, USART_PARITY_ODD, USART_STOP_1_BIT},    // SERIAL_7O1
    {USART_DATA_8BITS, USART_PARITY_ODD, USART_STOP_2_BIT},    // SERIAL_7O2
    {USART_DATA_8BITS, USART_PARITY_EVEN, USART_STOP_0_5_BIT}, // SERIAL_7E0_5
    {USART_DATA_8BITS, USART_PARITY_EVEN, USART_STOP_1_5_BIT}, // SERIAL_7E1_5
    {USART_DATA_8BITS, USART_PARITY_ODD, USART_STOP_0_5_BIT},  // SERIAL_7O0_5
    {USART_DATA_8BITS, USART_PARITY_ODD, USART_STOP_1_5_BIT},  // SERIAL_7O1_5

    {USART_DATA_8BITS, USART_PARITY_NONE, USART_STOP_1_BIT},   // SERIAL_8N1
    {USART_DATA_8BITS, USART_PARITY_NONE, USART_STOP_2_BIT},   // SERIAL_8N2
    {USART_DATA_9BITS, USART_PARITY_EVEN, USART_STOP_1_BIT},   // SERIAL_8E1
    {USART_DATA_9BITS, USART_PARITY_EVEN, USART_STOP_2_BIT},   // SERIAL_8E2
    {USART_DATA_9BITS, USART_PARITY_ODD,  USART_STOP_1_BIT},   // SERIAL_8O1
    {USART_DATA_9BITS, USART_PARITY_ODD,  USART_STOP_2_BIT},   // SERIAL_8O2
    {USART_DATA_8BITS, USART_PARITY_NONE, USART_STOP_0_5_BIT}, // SERIAL_8N0_5
    {USART_DATA_8BITS, USART_PARITY_NONE, USART_STOP_1_5_BIT}, // SERIAL_8N1_5
    {USART_DATA_9BITS, USART_PARITY_EVEN, USART_STOP_0_5_BIT}, // SERIAL_8E0_5
    {USART_DATA_9BITS, USART_PARITY_EVEN, USART_STOP_1_5_BIT}, // SERIAL_8E1_5
    {USART_DATA_9BITS, USART_PARITY_ODD,  USART_STOP_0_5_BIT}, // SERIAL_8O0_5
    {USART_DATA_9BITS, USART_PARITY_ODD,  USART_STOP_1_5_BIT}, // SERIAL_8O1_5

    {USART_DATA_9BITS, USART_PARITY_NONE, USART_STOP_1_BIT},   // SERIAL_9N1
    {USART_DATA_9BITS, USART_PARITY_NONE, USART_STOP_2_BIT},   // SERIAL_9N2
    {USART_DATA_9BITS, USART_PARITY_NONE, USART_STOP_0_5_BIT}, // SERIAL_9N0_5
    {USART_DATA_9BITS, USART_PARITY_NONE, USART_STOP_1_5_BIT}, // SERIAL_9N1_5
};
/**
  * @brief  usart object function
  * @param  usart address
  * @retval none
  */
HardwareSerial::HardwareSerial(usart_type* usart)
    : _USARTx(usart)
    , _callbackFunction(NULL)
    , _rxBufferHead(0)
    , _rxBufferTail(0)
{
    memset(_rxBuffer, 0, sizeof(_rxBuffer));
}

/**
  * @brief  串口中断入口
  * @param  无
  * @retval 无
  */
void HardwareSerial::IRQHandler()
{
    if(usart_flag_get(_USARTx, USART_RDBF_FLAG) != RESET)
    {
        uint8_t c = usart_data_receive(_USARTx);
        uint16_t i = (uint16_t)(_rxBufferHead + 1) % SERIAL_RX_BUFFER_SIZE;
        if (i != _rxBufferTail)
        {
            _rxBuffer[_rxBufferHead] = c;
            _rxBufferHead = i;
        }

        if(_callbackFunction)
        {
            _callbackFunction(this);
        }

        usart_flag_clear(_USARTx, USART_RDBF_FLAG);
    }
}

/**
  * @brief  串口初始化
  * @param  BaudRate: 波特率
  * @param  Config: 配置参数
  * @param  PreemptionPriority: 抢占优先级
  * @param  SubPriority: 从优先级
  * @retval 无
  */
void HardwareSerial::begin(
    uint32_t baudRate,
    SERIAL_Config_t config,
    uint8_t preemptionPriority,
    uint8_t subPriority
)
{
    gpio_type *GPIOx;
    gpio_init_type gpio_init_struct;
    uint16_t Tx_Pin, Rx_Pin;
    IRQn_Type USARTx_IRQn;

    if(_USARTx == SERIAL_1_USART)
    {   
        USARTx_IRQn = SERIAL_1_IRQ;
        crm_periph_clock_enable(SERIAL_1_CLOCK, TRUE);
        #ifndef IOMUX_NEW
        crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);        
        GPIOx = GPIOA;
        Tx_Pin = GPIO_Pin_9;
        Rx_Pin = GPIO_Pin_10;
        #else
        pinmap_pinout(SERIAL_1_USART_tx, PinMap_USART_TX);
        pinmap_pinout(SERIAL_1_USART_rx, PinMap_USART_RX);
        #endif
    }
    else if(_USARTx == SERIAL_2_USART)
    {
        USARTx_IRQn = SERIAL_2_IRQ;
        crm_periph_clock_enable(SERIAL_2_CLOCK, TRUE);

        #ifndef IOMUX_NEW
        crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
        GPIOx = GPIOA;
        Tx_Pin = GPIO_Pin_2;
        Rx_Pin = GPIO_Pin_3;
        #else
        pinmap_pinout(SERIAL_2_USART_tx, PinMap_USART_TX);
        pinmap_pinout(SERIAL_2_USART_rx, PinMap_USART_RX);
        #endif
    }
    else if(_USARTx == SERIAL_3_USART)
    {

        USARTx_IRQn = SERIAL_3_IRQ;
        crm_periph_clock_enable(SERIAL_3_CLOCK, TRUE);

        #ifndef IOMUX_NEW
        crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);        
        GPIOx = GPIOB;
        Tx_Pin = GPIO_Pin_10;
        Rx_Pin = GPIO_Pin_11;        
        #else
        pinmap_pinout(SERIAL_3_USART_tx, PinMap_USART_TX);
        pinmap_pinout(SERIAL_3_USART_rx, PinMap_USART_RX);       
        #endif
    }
    #ifdef SERIAL_4_USART
    else if(_USARTx == SERIAL_4_USART)
    {
        USARTx_IRQn = SERIAL_4_IRQ;
        crm_periph_clock_enable(SERIAL_4_CLOCK, TRUE);

        #ifndef IOMUX_NEW
        crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);   
        crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
        gpio_pin_remap_config(UART4_GMUX_0010, TRUE);             
        GPIOx = GPIOA;
        Tx_Pin = GPIO_Pin_0;
        Rx_Pin = GPIO_Pin_1;
        #else
        pinmap_pinout(SERIAL_4_USART_tx, PinMap_USART_TX);
        pinmap_pinout(SERIAL_4_USART_rx, PinMap_USART_RX);  
        #endif
    }
    #endif
    #ifdef SERIAL_5_USART
    else if(_USARTx == SERIAL_5_USART)
    {
        USARTx_IRQn = SERIAL_5_IRQ;
        crm_periph_clock_enable(SERIAL_5_CLOCK, TRUE);

        #ifndef IOMUX_NEW
        crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);  
        crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
        gpio_pin_remap_config(UART5_GMUX_0001, TRUE);           
        GPIOx = GPIOB;
        Tx_Pin = GPIO_Pin_9;
        Rx_Pin = GPIO_Pin_8;
        #else
        pinmap_pinout(SERIAL_5_USART_tx, PinMap_USART_TX);
        pinmap_pinout(SERIAL_5_USART_rx, PinMap_USART_RX);  
        #endif
    }
    #endif
    #ifdef SERIAL_6_USART
    else if (_USARTx == SERIAL_6_USART)
    {
        USARTx_IRQn = SERIAL_6_IRQ;
        crm_periph_clock_enable(SERIAL_6_CLOCK, TRUE);

        #ifndef IOMUX_NEW
        crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);    
        crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
        gpio_pin_remap_config(USART6_GMUX, TRUE);           
        GPIOx = GPIOA;
        Tx_Pin = GPIO_Pin_4;
        Rx_Pin = GPIO_Pin_5;
        #else
        pinmap_pinout(SERIAL_6_USART_tx, PinMap_USART_TX);
        pinmap_pinout(SERIAL_6_USART_rx, PinMap_USART_RX);  
        #endif
    }
    #endif
    #ifdef SERIAL_7_USART
    else if (_USARTx == SERIAL_7_USART)
    {
        USARTx_IRQn = SERIAL_7_IRQ;
        crm_periph_clock_enable(SERIAL_7_CLOCK, TRUE);

        #ifndef IOMUX_NEW
        crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);     
        crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
        gpio_pin_remap_config(UART7_GMUX, TRUE);          
        GPIOx = GPIOB;
        Tx_Pin = GPIO_Pin_4;
        Rx_Pin = GPIO_Pin_3;
        #else
        pinmap_pinout(SERIAL_7_USART_tx, PinMap_USART_TX);
        pinmap_pinout(SERIAL_7_USART_rx, PinMap_USART_RX);  
        #endif
    }
    #endif
    #ifdef SERIAL_8_USART
    else if (_USARTx == SERIAL_8_USART)
    {
        USARTx_IRQn = SERIAL_8_IRQ;
        crm_periph_clock_enable(SERIAL_8_CLOCK, TRUE);

        #ifndef IOMUX_NEW
        crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);    
        crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
        gpio_pin_remap_config(UART8_GMUX, TRUE);        
        GPIOx = GPIOC;
        Tx_Pin = GPIO_Pin_2;
        Rx_Pin = GPIO_Pin_3;
        #else
        pinmap_pinout(SERIAL_8_USART_tx, PinMap_USART_TX);
        pinmap_pinout(SERIAL_8_USART_rx, PinMap_USART_RX);  
        #endif
    }
    #endif
    else
    {
        return;
    }
    
    #ifndef IOMUX_NEW
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
	gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
	gpio_init_struct.gpio_pins = Tx_Pin;
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
	gpio_init(GPIOx, &gpio_init_struct);
	
	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
	gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
	gpio_init_struct.gpio_pins = Rx_Pin;
	gpio_init_struct.gpio_pull = GPIO_PULL_UP;
	gpio_init(GPIOx, &gpio_init_struct);
    #endif


    usart_init(_USARTx, baudRate, SERIAL_ConfigGrp[config].data_bit, SERIAL_ConfigGrp[config].stop_bit);
    usart_parity_selection_config(_USARTx, SERIAL_ConfigGrp[config].parity_selection);
    usart_transmitter_enable(_USARTx, TRUE);
    usart_receiver_enable(_USARTx, TRUE);

    nvic_irq_enable(USARTx_IRQn, preemptionPriority, subPriority);
    usart_interrupt_enable(_USARTx, USART_RDBF_INT, TRUE);

    usart_enable(_USARTx, TRUE);
}

/**
  * @brief  关闭串口
  * @param  无
  * @retval 无
  */
void HardwareSerial::end(void)
{
	usart_interrupt_enable(_USARTx, USART_RDBF_INT, FALSE);
    usart_enable(_USARTx, FALSE);
}

/**
  * @brief  串口中断回调
  * @param  Function: 回调函数
  * @retval 无
  */
void HardwareSerial::attachInterrupt(CallbackFunction_t func)
{
    _callbackFunction = func;
}

/**
  * @brief  获取可从串行端口读取的字节数
  * @param  无
  * @retval 可读取的字节数
  */
int HardwareSerial::available(void)
{
    return ((unsigned int)(SERIAL_RX_BUFFER_SIZE + _rxBufferHead - _rxBufferTail)) % SERIAL_RX_BUFFER_SIZE;
}

/**
  * @brief  读取传入的串行数据(字符)
  * @param  无
  * @retval 可用的传入串行数据的第一个字节 (如果没有可用的数据, 则为-1)
  */
int HardwareSerial::read(void)
{
    // if the head isn't ahead of the tail, we don't have any characters
    if (_rxBufferHead == _rxBufferTail)
    {
        return -1;
    }
    else
    {
        uint8_t c = _rxBuffer[_rxBufferTail];
        _rxBufferTail = (uint16_t)(_rxBufferTail + 1) % SERIAL_RX_BUFFER_SIZE;
        return c;
    }
}

/**
  * @brief  返回传入串行数据的下一个字节(字符), 而不将其从内部串行缓冲区中删除
  * @param  无
  * @retval 可用的传入串行数据的第一个字节 (如果没有可用的数据, 则为-1)
  */
int HardwareSerial::peek(void)
{
    if (_rxBufferHead == _rxBufferTail)
    {
        return -1;
    }
    else
    {
        return _rxBuffer[_rxBufferTail];
    }
}

/**
  * @brief  清空串口缓存
  * @param  无
  * @retval 无
  */
void HardwareSerial::flush(void)
{
    _rxBufferHead = _rxBufferTail;
}

/**
  * @brief  串口写入一个字节
  * @param  写入的字节
  * @retval 字节
  */
size_t HardwareSerial::write(uint8_t n)
{
    while(!usart_flag_get(_USARTx, USART_TDBE_FLAG)) {};
    usart_data_transmit(_USARTx, n);
    return 1;
}

#if SERIAL_1_ENABLE
HardwareSerial Serial(SERIAL_1_USART);

extern "C" SERIAL_1_IRQ_HANDLER_DEF()
{
    Serial.IRQHandler();
}
#endif

#if SERIAL_2_ENABLE
HardwareSerial Serial2(SERIAL_2_USART);

extern "C" SERIAL_2_IRQ_HANDLER_DEF()
{
    Serial2.IRQHandler();
}
#endif


#ifdef L021_series

#if SERIAL_3_ENABLE
HardwareSerial Serial3(SERIAL_3_USART);
#endif
#if SERIAL_4_ENABLE
HardwareSerial Serial4(SERIAL_4_USART);
#endif

void USART4_3_IRQHandler(void)
{
    #if SERIAL_3_ENABLE
    Serial3.IRQHandler();
    #endif
    #if SERIAL_4_ENABLE
    Serial4.IRQHandler();
    #endif   
}

#else

#if SERIAL_3_ENABLE
HardwareSerial Serial3(SERIAL_3_USART);

extern "C" SERIAL_3_IRQ_HANDLER_DEF()
{
    Serial3.IRQHandler();
}
#endif

#if SERIAL_4_ENABLE
HardwareSerial Serial4(SERIAL_4_USART);

extern "C" SERIAL_4_IRQ_HANDLER_DEF()
{
    Serial4.IRQHandler();
}
#endif
#endif

#if SERIAL_5_ENABLE
HardwareSerial Serial5(SERIAL_5_USART);

extern "C" SERIAL_5_IRQ_HANDLER_DEF()
{
    Serial5.IRQHandler();
}
#endif

#if SERIAL_6_ENABLE
HardwareSerial Serial6(SERIAL_6_USART);

extern "C" SERIAL_6_IRQ_HANDLER_DEF()
{
    Serial6.IRQHandler();
}
#endif

#if SERIAL_7_ENABLE
HardwareSerial Serial7(SERIAL_7_USART);

extern "C" SERIAL_7_IRQ_HANDLER_DEF()
{
    Serial7.IRQHandler();
}
#endif

#if SERIAL_8_ENABLE
HardwareSerial Serial8(SERIAL_8_USART);

extern "C" SERIAL_8_IRQ_HANDLER_DEF()
{
    Serial8.IRQHandler();
}
#endif
