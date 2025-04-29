/*
 * MIT License
 * Copyright (c) 2019 _VIFEXTech
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
#ifndef __HARDWAREI2C_H
#define __HARDWAREI2C_H

#include <stdint.h>
#include "mcu_type.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_EVENT_CHECK_NONE             ((uint32_t)0x00000000)    /*!< check flag none */
#define I2C_EVENT_CHECK_ACKFAIL          ((uint32_t)0x00000001)    /*!< check flag ackfail */
#define I2C_EVENT_CHECK_STOP             ((uint32_t)0x00000002)    /*!< check flag stop */

typedef enum
{
  I2C_MEM_ADDR_WIDIH_8                   = 0x01, /*!< memory address is 8 bit */
  I2C_MEM_ADDR_WIDIH_16                  = 0x02, /*!< memory address is 16 bit */
} i2c_mem_address_width_type;

typedef enum
{
  I2C_INT_MA_TX = 0,
  I2C_INT_MA_RX,
  I2C_INT_SLA_TX,
  I2C_INT_SLA_RX,
  I2C_DMA_MA_TX,
  I2C_DMA_MA_RX,
  I2C_DMA_SLA_TX,
  I2C_DMA_SLA_RX,
} i2c_mode_type;

typedef enum
{
  I2C_OK = 0,          /*!< no error */
  I2C_ERR_STEP_1,      /*!< step 1 error */
  I2C_ERR_STEP_2,      /*!< step 2 error */
  I2C_ERR_STEP_3,      /*!< step 3 error */
  I2C_ERR_STEP_4,      /*!< step 4 error */
  I2C_ERR_STEP_5,      /*!< step 5 error */
  I2C_ERR_STEP_6,      /*!< step 6 error */
  I2C_ERR_STEP_7,      /*!< step 7 error */
  I2C_ERR_STEP_8,      /*!< step 8 error */
  I2C_ERR_STEP_9,      /*!< step 9 error */
  I2C_ERR_STEP_10,     /*!< step 10 error */
  I2C_ERR_STEP_11,     /*!< step 11 error */
  I2C_ERR_STEP_12,     /*!< step 12 error */
  I2C_ERR_TCRLD,       /*!< tcrld error */
  I2C_ERR_TDC,         /*!< tdc error */
  I2C_ERR_ADDR,        /*!< addr error */
  I2C_ERR_STOP,        /*!< stop error */
  I2C_ERR_ACKFAIL,     /*!< ackfail error */
  I2C_ERR_TIMEOUT,     /*!< timeout error */
  I2C_ERR_INTERRUPT,   /*!< interrupt error */
} i2c_status_type;

typedef struct
{
  i2c_type                               *i2cx;                   /*!< i2c registers base address      */
  uint8_t                                *pbuff;                  /*!< pointer to i2c transfer buffer  */
  __IO uint16_t                          psize;                   /*!< i2c transfer size               */
  __IO uint16_t                          pcount;                  /*!< i2c transfer counter            */
  __IO uint32_t                          mode;                    /*!< i2c communication mode          */
  __IO uint32_t                          status;                  /*!< i2c communication status        */
  __IO i2c_status_type                   error_code;              /*!< i2c error code                  */
  dma_channel_type                       *dma_tx_channel;         /*!< dma transmit channel            */
  dma_channel_type                       *dma_rx_channel;         /*!< dma receive channel             */
  dma_init_type                          dma_init_struct;         /*!< dma init parameters             */
} i2c_handle_type;

extern i2c_handle_type  I2C_1;
extern i2c_handle_type  I2C_2;


/* I2C START mask */
//#define I2C_CTRL1_STARTGEN          ((uint16_t)0x0100)
/* I2C STOP mask */
//#define I2C_CTRL1_STOPGEN           ((uint16_t)0x0200)
//void I2Cx_Init(i2c_type* I2Cx);
//void I2Cx_Read(i2c_type *I2Cx, uint8_t slaveAddr, uint8_t regAddr, void *buf, uint32_t length);
//void I2Cx_Write(i2c_type *I2Cx, uint8_t slaveAddr, uint8_t regAddr, void *buf, uint32_t length);
//uint8_t I2Cx_ReadReg(i2c_type *I2Cx, uint8_t slaveAddr, uint8_t regAddr);
//void I2Cx_WriteReg(i2c_type *I2Cx, uint8_t slaveAddr, uint8_t regAddr, uint8_t value);

void            i2c_config                (i2c_handle_type* hi2c);
void            i2c_lowlevel_init         (i2c_handle_type* hi2c);
void            i2c_reset_ctrl2_register  (i2c_handle_type* hi2c);
i2c_status_type i2c_wait_end              (i2c_handle_type* hi2c, uint32_t timeout);
i2c_status_type i2c_wait_flag             (i2c_handle_type* hi2c, uint32_t flag, uint32_t event_check, uint32_t timeout);

i2c_status_type i2c_master_transmit       (i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_master_receive        (i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_slave_transmit        (i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_slave_receive         (i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout);

i2c_status_type i2c_master_transmit_int   (i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_master_receive_int    (i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_slave_transmit_int    (i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_slave_receive_int     (i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout);

i2c_status_type i2c_master_transmit_dma   (i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_master_receive_dma    (i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_slave_transmit_dma    (i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_slave_receive_dma     (i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout);

i2c_status_type i2c_smbus_master_transmit (i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_smbus_master_receive  (i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_smbus_slave_transmit  (i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_smbus_slave_receive   (i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout);

i2c_status_type i2c_memory_write          (i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_memory_write_int      (i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_memory_write_dma      (i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_memory_read           (i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_memory_read_int       (i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_memory_read_dma       (i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout);

void            i2c_evt_irq_handler       (i2c_handle_type* hi2c);
void            i2c_err_irq_handler       (i2c_handle_type* hi2c);
void            i2c_dma_tx_irq_handler    (i2c_handle_type* hi2c);
void            i2c_dma_rx_irq_handler    (i2c_handle_type* hi2c);


#ifdef __cplusplus
}
#endif


#endif

