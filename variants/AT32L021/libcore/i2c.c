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
#include "Arduino.h"
#include "i2c.h"

i2c_handle_type  I2C_1 = {.i2cx = I2C1};
i2c_handle_type  I2C_2 = {.i2cx = I2C2};

#define DMA_GET_REQUEST(DMA_CHANNEL) \
(((uint32_t)(DMA_CHANNEL) == ((uint32_t)hi2c->dma_tx_channel)) ? I2C_DMA_REQUEST_TX : I2C_DMA_REQUEST_RX)

#define DMA_GET_TC_FLAG(DMA_CHANNEL) \
(((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL1))? DMA1_FDT1_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL2))? DMA1_FDT2_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL3))? DMA1_FDT3_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL4))? DMA1_FDT4_FLAG : \
                                                         DMA1_FDT5_FLAG)

#define DMA_GET_HT_FLAG(DMA_CHANNEL) \
(((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL1))? DMA1_HDT1_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL2))? DMA1_HDT2_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL3))? DMA1_HDT3_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL4))? DMA1_HDT4_FLAG : \
                                                         DMA1_HDT5_FLAG)

#define DMA_GET_TERR_FLAG(DMA_CHANNEL) \
(((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL1))? DMA1_DTERR1_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL2))? DMA1_DTERR2_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL3))? DMA1_DTERR3_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL4))? DMA1_DTERR4_FLAG : \
                                                         DMA1_DTERR5_FLAG)

#define I2C_START                        0
#define I2C_END                          1

/**
  * @brief  initializes peripherals used by the i2c.
  * @param  none
  * @retval none
  */
void i2c_lowlevel_init(i2c_handle_type* hi2c)
{
  gpio_init_type gpio_init_structure;

  if(hi2c->i2cx == I2C1)
  {
    /* i2c periph clock enable */
    crm_periph_clock_enable(CRM_I2C1_PERIPH_CLOCK, TRUE);

    pinmap_pinout(I2C_1_SCL, PinMap_I2C_SCL);
    pinmap_pinout(I2C_1_SDA, PinMap_I2C_SDA);

    /* configure and enable i2c interrupt */
    nvic_irq_enable(I2C_1_IRQn, 0, 0);

    /* configure and enable i2c dma channel interrupt */
    nvic_irq_enable(I2C_1_DMA_TX_IRQn, 0, 0);
    nvic_irq_enable(I2C_1_DMA_RX_IRQn, 0, 0);

    /* i2c dma tx and rx channels configuration */
    /* enable the dma clock */
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

    /* i2c dma channel configuration */
    hi2c->dma_tx_channel = I2C_1_DMA_TX_Channel;
    hi2c->dma_rx_channel = I2C_1_DMA_RX_Channel;

    dma_reset(hi2c->dma_tx_channel);
    dma_reset(hi2c->dma_rx_channel);

    hi2c->dma_init_struct.peripheral_base_addr    = (uint32_t)&hi2c->i2cx->txdt;
    hi2c->dma_init_struct.memory_base_addr        = 0;
    hi2c->dma_init_struct.direction               = DMA_DIR_MEMORY_TO_PERIPHERAL;
    hi2c->dma_init_struct.buffer_size             = 0xFFFF;
    hi2c->dma_init_struct.peripheral_inc_enable   = FALSE;
    hi2c->dma_init_struct.memory_inc_enable       = TRUE;
    hi2c->dma_init_struct.peripheral_data_width   = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    hi2c->dma_init_struct.memory_data_width       = DMA_MEMORY_DATA_WIDTH_BYTE;
    hi2c->dma_init_struct.loop_mode_enable        = FALSE;
    hi2c->dma_init_struct.priority                = DMA_PRIORITY_LOW;

    dma_init(hi2c->dma_tx_channel, &hi2c->dma_init_struct);
    dma_init(hi2c->dma_rx_channel, &hi2c->dma_init_struct);

    dma_flexible_config(DMA1, I2C_1_DMA_TX_DMAMUX_Channel, I2C_1_DMA_TX_DMAREQ);
    dma_flexible_config(DMA1, I2C_1_DMA_RX_DMAMUX_Channel, I2C_1_DMA_RX_DMAREQ);

    /* config i2c */
    i2c_init(hi2c->i2cx, 0x0F, I2C_1_CLKCTRL);

    i2c_own_address1_set(hi2c->i2cx, I2C_ADDRESS_MODE_7BIT, I2C_1_ADDRESS);
  }
  else if(hi2c->i2cx == I2C2)
  {
    /* i2c periph clock enable */
    crm_periph_clock_enable(CRM_I2C2_PERIPH_CLOCK, TRUE);

    pinmap_pinout(I2C_2_SCL, PinMap_I2C_SCL);
    pinmap_pinout(I2C_2_SDA, PinMap_I2C_SDA);

    /* configure and enable i2c interrupt */
    nvic_irq_enable(I2C_2_IRQn, 0, 0);

    /* configure and enable i2c dma channel interrupt */
    nvic_irq_enable(I2C_2_DMA_TX_IRQn, 0, 0);
    nvic_irq_enable(I2C_2_DMA_RX_IRQn, 0, 0);

    /* i2c dma tx and rx channels configuration */
    /* enable the dma clock */
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

    /* i2c dma channel configuration */
    hi2c->dma_tx_channel = I2C_2_DMA_TX_Channel;
    hi2c->dma_rx_channel = I2C_2_DMA_RX_Channel;

    dma_reset(hi2c->dma_tx_channel);
    dma_reset(hi2c->dma_rx_channel);

    hi2c->dma_init_struct.peripheral_base_addr    = (uint32_t)&hi2c->i2cx->txdt;
    hi2c->dma_init_struct.memory_base_addr        = 0;
    hi2c->dma_init_struct.direction               = DMA_DIR_MEMORY_TO_PERIPHERAL;
    hi2c->dma_init_struct.buffer_size             = 0xFFFF;
    hi2c->dma_init_struct.peripheral_inc_enable   = FALSE;
    hi2c->dma_init_struct.memory_inc_enable       = TRUE;
    hi2c->dma_init_struct.peripheral_data_width   = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    hi2c->dma_init_struct.memory_data_width       = DMA_MEMORY_DATA_WIDTH_BYTE;
    hi2c->dma_init_struct.loop_mode_enable        = FALSE;
    hi2c->dma_init_struct.priority                = DMA_PRIORITY_LOW;

    dma_init(hi2c->dma_tx_channel, &hi2c->dma_init_struct);
    dma_init(hi2c->dma_rx_channel, &hi2c->dma_init_struct);

    dma_flexible_config(DMA1, I2C_2_DMA_TX_DMAMUX_Channel, I2C_2_DMA_TX_DMAREQ);
    dma_flexible_config(DMA1, I2C_2_DMA_RX_DMAMUX_Channel, I2C_2_DMA_RX_DMAREQ);

    /* config i2c */
    i2c_init(hi2c->i2cx, 0x0F, I2C_2_CLKCTRL);

    i2c_own_address1_set(hi2c->i2cx, I2C_ADDRESS_MODE_7BIT, I2C_2_ADDRESS);
  }
}

/**
  * @brief  i2c peripheral initialization.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_config(i2c_handle_type* hi2c)
{
  /* reset i2c peripheral */
  i2c_reset(hi2c->i2cx);

  /* i2c peripheral initialization */
  i2c_lowlevel_init(hi2c);

  /* i2c peripheral enable */
  i2c_enable(hi2c->i2cx, TRUE);
}

/**
  * @brief  refresh i2c register.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_refresh_txdt_register(i2c_handle_type* hi2c)
{
  /* clear tdis flag */
  if (i2c_flag_get(hi2c->i2cx, I2C_TDIS_FLAG) != RESET)
  {
    hi2c->i2cx->txdt = 0x00;
  }

  /* refresh txdt register*/
  if (i2c_flag_get(hi2c->i2cx, I2C_TDBE_FLAG) == RESET)
  {
    hi2c->i2cx->sts_bit.tdbe = 1;
  }
}

/**
  * @brief  reset ctrl2 register.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_reset_ctrl2_register(i2c_handle_type* hi2c)
{
  hi2c->i2cx->ctrl2_bit.saddr   = 0;
  hi2c->i2cx->ctrl2_bit.readh10 = 0;
  hi2c->i2cx->ctrl2_bit.cnt     = 0;
  hi2c->i2cx->ctrl2_bit.rlden   = 0;
  hi2c->i2cx->ctrl2_bit.dir     = 0;
}

/**
  * @brief  wait for the transfer to end.
  * @param  hi2c: the handle points to the operation information.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_wait_end(i2c_handle_type* hi2c, uint32_t timeout)
{
  while(hi2c->status != I2C_END)
  {
    /* check timeout */
    if((timeout--) == 0)
    {
      return I2C_ERR_TIMEOUT;
    }
  }

  if(hi2c->error_code != I2C_OK)
  {
    return hi2c->error_code;
  }

  return I2C_OK;
}

/**
  * @brief  wait for the flag to be set or reset, only BUSYF flag
  *         is waiting to be reset, and other flags are waiting to be set
  * @param  hi2c: the handle points to the operation information.
  * @param  flag: specifies the flag to check.
  *         this parameter can be one of the following values:
  *         - I2C_TDBE_FLAG: transmit data buffer empty flag.
  *         - I2C_TDIS_FLAG: send interrupt status.
  *         - I2C_RDBF_FLAG: receive data buffer full flag.
  *         - I2C_ADDRF_FLAG: 0~7 bit address match flag.
  *         - I2C_ACKFAIL_FLAG: acknowledge failure flag.
  *         - I2C_STOPF_FLAG: stop condition generation complete flag.
  *         - I2C_TDC_FLAG: transmit data complete flag.
  *         - I2C_TCRLD_FLAG: transmission is complete, waiting to load data.
  *         - I2C_BUSERR_FLAG: bus error flag.
  *         - I2C_ARLOST_FLAG: arbitration lost flag.
  *         - I2C_OUF_FLAG: overflow or underflow flag.
  *         - I2C_PECERR_FLAG: pec receive error flag.
  *         - I2C_TMOUT_FLAG: smbus timeout flag.
  *         - I2C_ALERTF_FLAG: smbus alert flag.
  *         - I2C_BUSYF_FLAG: bus busy flag transmission mode.
  *         - I2C_SDIR_FLAG: slave data transmit direction.
  * @param  event_check: check other error flags while waiting for the flag.
  *         parameter as following values:
  *         - I2C_EVENT_CHECK_NONE
  *         - I2C_EVENT_CHECK_ACKFAIL
  *         - I2C_EVENT_CHECK_STOP
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_wait_flag(i2c_handle_type* hi2c, uint32_t flag, uint32_t event_check, uint32_t timeout)
{
  if(flag == I2C_BUSYF_FLAG)
  {
    while(i2c_flag_get(hi2c->i2cx, flag) != RESET)
    {
      /* check timeout */
      if((timeout--) == 0)
      {
        hi2c->error_code = I2C_ERR_TIMEOUT;

        return I2C_ERR_TIMEOUT;
      }
    }
  }
  else
  {
    while(i2c_flag_get(hi2c->i2cx, flag) == RESET)
    {
      /* check the ack fail flag */
      if(event_check & I2C_EVENT_CHECK_ACKFAIL)
      {
        if(i2c_flag_get(hi2c->i2cx, I2C_ACKFAIL_FLAG) != RESET)
        {
          /* clear ack fail flag */
          i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);

          hi2c->error_code = I2C_ERR_ACKFAIL;

          return I2C_ERR_ACKFAIL;
        }
      }

      /* check the stop flag */
      if(event_check & I2C_EVENT_CHECK_STOP)
      {
        if(i2c_flag_get(hi2c->i2cx, I2C_STOPF_FLAG) != RESET)
        {
          /* clear stop flag */
          i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

          i2c_reset_ctrl2_register(hi2c);

          hi2c->error_code = I2C_ERR_STOP;

          return I2C_ERR_STOP;
        }
      }

      /* check timeout */
      if((timeout--) == 0)
      {
        hi2c->error_code = I2C_ERR_TIMEOUT;

        return I2C_ERR_TIMEOUT;
      }
    }
  }

  return I2C_OK;
}

/**
  * @brief  dma transfer cofiguration.
  * @param  hi2c: the handle points to the operation information.
  * @param  dma_channelx: dma channel to be cofigured.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @retval none.
  */
void i2c_dma_config(i2c_handle_type* hi2c, dma_channel_type* dma_channel, uint8_t* pdata, uint16_t size)
{
  /* disable the dma channel */
  dma_channel_enable(dma_channel, FALSE);

  /* disable the transfer complete interrupt */
  dma_interrupt_enable(dma_channel, DMA_FDT_INT, FALSE);

  /* configure the dma channel with the buffer address and the buffer size */
  hi2c->dma_init_struct.memory_base_addr     = (uint32_t)pdata;
  hi2c->dma_init_struct.direction            = (dma_channel == hi2c->dma_tx_channel) ? DMA_DIR_MEMORY_TO_PERIPHERAL : DMA_DIR_PERIPHERAL_TO_MEMORY;
  hi2c->dma_init_struct.peripheral_base_addr = (dma_channel == hi2c->dma_tx_channel) ? (uint32_t)&hi2c->i2cx->txdt : (uint32_t)&hi2c->i2cx->rxdt;
  hi2c->dma_init_struct.buffer_size          = (uint32_t)size;
  dma_init(dma_channel, &hi2c->dma_init_struct);

  /* enable the transfer complete interrupt */
  dma_interrupt_enable(dma_channel, DMA_FDT_INT, TRUE);

  /* enable the dma channel */
  dma_channel_enable(dma_channel, TRUE);
}

/**
  * @brief  start transfer in poll mode or interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  start: config gen start condition mode.
  *         parameter as following values:
  *         - I2C_WITHOUT_START: transfer data without start condition.
  *         - I2C_GEN_START_READ: read data and generate start.
  *         - I2C_GEN_START_WRITE: send data and generate start.
  * @retval i2c status.
  */
void i2c_start_transfer(i2c_handle_type* hi2c, uint16_t address, i2c_start_mode_type start)
{
  if (hi2c->pcount > MAX_TRANSFER_CNT)
  {
    hi2c->psize = MAX_TRANSFER_CNT;

    i2c_transmit_set(hi2c->i2cx, address, hi2c->psize, I2C_RELOAD_MODE, start);
  }
  else
  {
    hi2c->psize = hi2c->pcount;

    i2c_transmit_set(hi2c->i2cx, address, hi2c->psize, I2C_AUTO_STOP_MODE, start);
  }
}

/**
  * @brief  start transfer in dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  start: config gen start condition mode.
  *         parameter as following values:
  *         - I2C_WITHOUT_START: transfer data without start condition.
  *         - I2C_GEN_START_READ: read data and generate start.
  *         - I2C_GEN_START_WRITE: send data and generate start.
  * @retval i2c status.
  */
void i2c_start_transfer_dma(i2c_handle_type* hi2c, dma_channel_type* dma_channelx, uint16_t address, i2c_start_mode_type start)
{
  if (hi2c->pcount > MAX_TRANSFER_CNT)
  {
    hi2c->psize = MAX_TRANSFER_CNT;

    /* config dma */
    i2c_dma_config(hi2c, dma_channelx, hi2c->pbuff, hi2c->psize);

    i2c_transmit_set(hi2c->i2cx, address, hi2c->psize, I2C_RELOAD_MODE, start);
  }
  else
  {
    hi2c->psize = hi2c->pcount;

    /* config dma */
    i2c_dma_config(hi2c, dma_channelx, hi2c->pbuff, hi2c->psize);

    i2c_transmit_set(hi2c->i2cx, address, hi2c->psize, I2C_AUTO_STOP_MODE, start);
  }
}

/**
  * @brief  the master transmits data through polling mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_master_transmit(i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->pbuff = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* start transfer */
  i2c_start_transfer(hi2c, address, I2C_GEN_START_WRITE);

  while (hi2c->pcount > 0)
  {
    /* wait for the tdis flag to be set */
    if(i2c_wait_flag(hi2c, I2C_TDIS_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
      return I2C_ERR_STEP_2;
    }

    /* send data */
    i2c_data_send(hi2c->i2cx, *hi2c->pbuff++);
    hi2c->psize--;
    hi2c->pcount--;

    if ((hi2c->psize == 0) && (hi2c->pcount != 0))
    {
      /* wait for the tcrld flag to be set  */
      if (i2c_wait_flag(hi2c, I2C_TCRLD_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
      {
        return I2C_ERR_STEP_3;
      }

      /* continue transfer */
      i2c_start_transfer(hi2c, address, I2C_WITHOUT_START);
    }
  }

  /* wait for the stop flag to be set  */
  if(i2c_wait_flag(hi2c, I2C_STOPF_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_4;
  }

  /* clear stop flag */
  i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

  /* reset ctrl2 register */
  i2c_reset_ctrl2_register(hi2c);

  return I2C_OK;
}

/**
  * @brief  the slave receive data through polling mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_slave_receive(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->pbuff = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* enable acknowledge */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* wait for the addr flag to be set */
  if (i2c_wait_flag(hi2c, I2C_ADDRF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_2;
  }

  /* clear addr flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ADDRF_FLAG);

  while (hi2c->pcount > 0)
  {
    /* wait for the rdbf flag to be set  */
    if(i2c_wait_flag(hi2c, I2C_RDBF_FLAG, I2C_EVENT_CHECK_STOP, timeout) != I2C_OK)
    {
      /* disable acknowledge */
      i2c_ack_enable(hi2c->i2cx, FALSE);

      /* if data is received, read data */
      if (i2c_flag_get(hi2c->i2cx, I2C_RDBF_FLAG) == SET)
      {
        /* read data */
        (*hi2c->pbuff++) = i2c_data_receive(hi2c->i2cx);
        hi2c->pcount--;
      }

      return I2C_ERR_STEP_4;
    }

    /* read data */
    (*hi2c->pbuff++) = i2c_data_receive(hi2c->i2cx);
    hi2c->pcount--;
  }

  /* wait for the stop flag to be set */
  if(i2c_wait_flag(hi2c, I2C_STOPF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    /* disable acknowledge */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    return I2C_ERR_STEP_5;
  }

  /* clear stop flag */
  i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    /* disable acknowledge */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    return I2C_ERR_STEP_6;
  }

  return I2C_OK;
}

/**
  * @brief  the master receive data through polling mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_master_receive(i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->pbuff = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* start transfer */
  i2c_start_transfer(hi2c, address, I2C_GEN_START_READ);

  while (hi2c->pcount > 0)
  {
    /* wait for the rdbf flag to be set  */
    if(i2c_wait_flag(hi2c, I2C_RDBF_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
      return I2C_ERR_STEP_2;
    }

    /* read data */
    (*hi2c->pbuff++) = i2c_data_receive(hi2c->i2cx);
    hi2c->pcount--;
    hi2c->psize--;

    if ((hi2c->psize == 0) && (hi2c->pcount != 0))
    {
      /* wait for the tcrld flag to be set  */
      if (i2c_wait_flag(hi2c, I2C_TCRLD_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
      {
        return I2C_ERR_STEP_3;
      }

      /* continue transfer */
      i2c_start_transfer(hi2c, address, I2C_WITHOUT_START);
    }
  }

  /* wait for the stop flag to be set  */
  if(i2c_wait_flag(hi2c, I2C_STOPF_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_4;
  }

  /* clear stop flag */
  i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

  /* reset ctrl2 register */
  i2c_reset_ctrl2_register(hi2c);

  return I2C_OK;
}

/**
  * @brief  the slave transmits data through polling mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_slave_transmit(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->pbuff = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* enable acknowledge */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* wait for the addr flag to be set */
  if (i2c_wait_flag(hi2c, I2C_ADDRF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    /* disable acknowledge */
    i2c_ack_enable(hi2c->i2cx, FALSE);
    return I2C_ERR_STEP_2;
  }

  /* clear addr flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ADDRF_FLAG);

  /* if 10-bit address mode is used */
  if (hi2c->i2cx->ctrl2_bit.addr10 != RESET)
  {
    /* wait for the addr flag to be set */
    if (i2c_wait_flag(hi2c, I2C_ADDRF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
    {
      /* disable acknowledge */
      i2c_ack_enable(hi2c->i2cx, FALSE);

      return I2C_ERR_STEP_3;
    }

    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDRF_FLAG);
  }

  while (hi2c->pcount > 0)
  {
    /* wait for the tdis flag to be set */
    if(i2c_wait_flag(hi2c, I2C_TDIS_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
      /* disable acknowledge */
      i2c_ack_enable(hi2c->i2cx, FALSE);

      return I2C_ERR_STEP_5;
    }

    /* send data */
    i2c_data_send(hi2c->i2cx, *hi2c->pbuff++);
    hi2c->pcount--;
  }

  /* wait for the ackfail flag to be set */
  if(i2c_wait_flag(hi2c, I2C_ACKFAIL_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_6;
  }

  /* clear ack fail flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);

  /* wait for the stop flag to be set */
  if(i2c_wait_flag(hi2c, I2C_STOPF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    /* disable acknowledge */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    return I2C_ERR_STEP_7;
  }

  /* clear stop flag */
  i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    /* disable acknowledge */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    return I2C_ERR_STEP_8;
  }

  /* refresh tx dt register */
  i2c_refresh_txdt_register(hi2c);

  return I2C_OK;
}

/**
  * @brief  the master transmits data through interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_master_transmit_int(i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_INT_MA_TX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* start transfer */
  i2c_start_transfer(hi2c, address, I2C_GEN_START_WRITE);

  /* enable interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_TDC_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_TD_INT, TRUE);

  return I2C_OK;
}

/**
  * @brief  the slave receive data through interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_slave_receive_int(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_INT_SLA_RX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* enable acknowledge */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* enable interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_TDC_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_ADDR_INT | I2C_RD_INT, TRUE);

  return I2C_OK;
}

/**
  * @brief  the master receive data through interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_master_receive_int(i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_INT_MA_RX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* start transfer */
  i2c_start_transfer(hi2c, address, I2C_GEN_START_READ);

  /* enable interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_TDC_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_RD_INT, TRUE);

  return I2C_OK;
}

/**
  * @brief  the slave transmits data through interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_slave_transmit_int(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_INT_SLA_TX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* enable acknowledge */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* enable interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_TDC_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_ADDR_INT | I2C_TD_INT, TRUE);

  i2c_refresh_txdt_register(hi2c);

  return I2C_OK;
}

/**
  * @brief  the master transmits data through dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_master_transmit_dma(i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_DMA_MA_TX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* disable dma request */
  i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_TX, FALSE);

  /* start transfer */
  i2c_start_transfer_dma(hi2c, hi2c->dma_tx_channel, address, I2C_GEN_START_WRITE);

  /* enable i2c interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_ACKFIAL_INT, TRUE);

  /* enable dma request */
  i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_TX, TRUE);

  return I2C_OK;
}

/**
  * @brief  the slave receive data through dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_slave_receive_dma(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_DMA_SLA_RX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* disable dma request */
  i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_RX, FALSE);

  /* config dma */
  i2c_dma_config(hi2c, hi2c->dma_rx_channel, hi2c->pbuff, size);

  /* enable acknowledge */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* enable i2c interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_ADDR_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_ERR_INT, TRUE);

  /* enable dma request */
  i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_RX, TRUE);

  return I2C_OK;
}

/**
  * @brief  the master receive data through dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_master_receive_dma(i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_DMA_MA_RX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* disable dma request */
  i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_RX, FALSE);

  /* start transfer */
  i2c_start_transfer_dma(hi2c, hi2c->dma_rx_channel, address, I2C_GEN_START_READ);

  /* enable i2c interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_ACKFIAL_INT, TRUE);

  /* enable dma request */
  i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_RX, TRUE);

  return I2C_OK;
}

/**
  * @brief  the slave transmits data through dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_slave_transmit_dma(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_DMA_SLA_TX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* disable dma request */
  i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_TX, FALSE);

  /* config dma */
  i2c_dma_config(hi2c, hi2c->dma_tx_channel, hi2c->pbuff, size);

  /* enable acknowledge */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* enable i2c interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_ADDR_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_ERR_INT, TRUE);

  /* enable dma request */
  i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_TX, TRUE);

  return I2C_OK;
}

/**
  * @brief  send memory address.
  * @param  hi2c: the handle points to the operation information.
  * @param  mem_address_width: memory address width.
  *         this parameter can be one of the following values:
  *         - I2C_MEM_ADDR_WIDIH_8:  memory address is 8 bit 
  *         - I2C_MEM_ADDR_WIDIH_16:  memory address is 16 bit 
  * @param  address: memory device address.
  * @param  mem_address: memory address.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_memory_address_send(i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t mem_address, int32_t timeout)
{
  i2c_status_type err_code;
  
  if(mem_address_width == I2C_MEM_ADDR_WIDIH_8)
  {
    /* send memory address */
    i2c_data_send(hi2c->i2cx, mem_address & 0xFF);
  }
  else
  {
    /* send memory address */
    i2c_data_send(hi2c->i2cx, (mem_address >> 8) & 0xFF);  
    
    /* wait for the tdis flag to be set */
    err_code = i2c_wait_flag(hi2c, I2C_TDIS_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout);
    
    if(err_code != I2C_OK)
    {
      return err_code;
    }
    
    /* send memory address */
    i2c_data_send(hi2c->i2cx, mem_address & 0xFF);  
  }
  
  return I2C_OK;
}

/**
  * @brief  write data to the memory device through polling mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  mem_address_width: memory address width.
  *         this parameter can be one of the following values:
  *         - I2C_MEM_ADDR_WIDIH_8:  memory address is 8 bit 
  *         - I2C_MEM_ADDR_WIDIH_16:  memory address is 16 bit 
  * @param  address: memory device address.
  * @param  mem_address: memory address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_memory_write(i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->pbuff = pdata;
  hi2c->pcount = size + mem_address_width;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* start transfer */
  i2c_start_transfer(hi2c, address, I2C_GEN_START_WRITE);

  /* wait for the tdis flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDIS_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_2;
  }

  /* send memory address */
  if(i2c_memory_address_send(hi2c, mem_address_width, mem_address, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_3;
  }

  hi2c->psize -= mem_address_width;
  hi2c->pcount -= mem_address_width;

  while (hi2c->pcount > 0)
  {
    /* wait for the tdis flag to be set */
    if(i2c_wait_flag(hi2c, I2C_TDIS_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
      return I2C_ERR_STEP_4;
    }

    /* send data */
    i2c_data_send(hi2c->i2cx, *hi2c->pbuff++);
    hi2c->psize--;
    hi2c->pcount--;

    if ((hi2c->psize == 0) && (hi2c->pcount != 0))
    {
      /* wait for the tcrld flag to be set  */
      if (i2c_wait_flag(hi2c, I2C_TCRLD_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
      {
        return I2C_ERR_STEP_5;
      }

      /* continue transfer */
      i2c_start_transfer(hi2c, address, I2C_WITHOUT_START);
    }
  }

  /* wait for the stop flag to be set  */
  if(i2c_wait_flag(hi2c, I2C_STOPF_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_6;
  }

  /* clear stop flag */
  i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

  /* reset ctrl2 register */
  i2c_reset_ctrl2_register(hi2c);

  return I2C_OK;
}

/**
  * @brief  read data from memory device through polling mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  mem_address_width: memory address width.
  *         this parameter can be one of the following values:
  *         - I2C_MEM_ADDR_WIDIH_8:  memory address is 8 bit 
  *         - I2C_MEM_ADDR_WIDIH_16:  memory address is 16 bit 
  * @param  address: memory device address.
  * @param  mem_address: memory address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_memory_read(i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->pbuff = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* start transfer */
  i2c_transmit_set(hi2c->i2cx, address, mem_address_width, I2C_SOFT_STOP_MODE, I2C_GEN_START_WRITE);

  /* wait for the tdis flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDIS_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_2;
  }

  /* send memory address */
  if(i2c_memory_address_send(hi2c, mem_address_width, mem_address, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_3;
  }

  /* wait for the tdc flag to be set */
  if (i2c_wait_flag(hi2c, I2C_TDC_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_4;
  }

  /* start transfer */
  i2c_start_transfer(hi2c, address, I2C_GEN_START_READ);

  while (hi2c->pcount > 0)
  {
    /* wait for the rdbf flag to be set  */
    if (i2c_wait_flag(hi2c, I2C_RDBF_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
      return I2C_ERR_STEP_5;
    }

    /* read data */
    (*hi2c->pbuff++) = i2c_data_receive(hi2c->i2cx);
    hi2c->pcount--;
    hi2c->psize--;

    if ((hi2c->psize == 0) && (hi2c->pcount != 0))
    {
      /* wait for the tcrld flag to be set  */
      if (i2c_wait_flag(hi2c, I2C_TCRLD_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
      {
        return I2C_ERR_STEP_6;
      }

      /* continue transfer */
      i2c_start_transfer(hi2c, address, I2C_WITHOUT_START);
    }
  }

  /* wait for the stop flag to be set  */
  if (i2c_wait_flag(hi2c, I2C_STOPF_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_7;
  }

  /* clear stop flag */
  i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

  /* reset ctrl2 register */
  i2c_reset_ctrl2_register(hi2c);

  return I2C_OK;
}

/**
  * @brief  write data to the memory device through interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  mem_address_width: memory address width.
  *         this parameter can be one of the following values:
  *         - I2C_MEM_ADDR_WIDIH_8:  memory address is 8 bit 
  *         - I2C_MEM_ADDR_WIDIH_16:  memory address is 16 bit 
  * @param  address: memory device address.
  * @param  mem_address: memory address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_memory_write_int(i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_INT_MA_TX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size + mem_address_width;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* start transfer */
  i2c_start_transfer(hi2c, address, I2C_GEN_START_WRITE);

  /* wait for the tdis flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDIS_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_2;
  }

  /* send memory address */
  if(i2c_memory_address_send(hi2c, mem_address_width, mem_address, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_3;
  }
  
  hi2c->psize--;
  hi2c->pcount--;

  /* enable interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_TDC_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_TD_INT, TRUE);

  return I2C_OK;
}

/**
  * @brief  read data from memory device through interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  mem_address_width: memory address width.
  *         this parameter can be one of the following values:
  *         - I2C_MEM_ADDR_WIDIH_8:  memory address is 8 bit 
  *         - I2C_MEM_ADDR_WIDIH_16:  memory address is 16 bit 
  * @param  address: memory device address.
  * @param  mem_address: memory address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_memory_read_int(i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_INT_MA_RX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* start transfer */
  i2c_transmit_set(hi2c->i2cx, address, mem_address_width, I2C_SOFT_STOP_MODE, I2C_GEN_START_WRITE);

  /* wait for the tdis flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDIS_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_2;
  }

  /* send memory address */
  if(i2c_memory_address_send(hi2c, mem_address_width, mem_address, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_3;
  }

  /* wait for the tdc flag to be set */
  if (i2c_wait_flag(hi2c, I2C_TDC_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_4;
  }

  /* start transfer */
  i2c_start_transfer(hi2c, address, I2C_GEN_START_READ);

  /* enable i2c interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_TDC_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_RD_INT, TRUE);

  return I2C_OK;
}

/**
  * @brief  write data to the memory device through dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  mem_address_width: memory address width.
  *         this parameter can be one of the following values:
  *         - I2C_MEM_ADDR_WIDIH_8:  memory address is 8 bit 
  *         - I2C_MEM_ADDR_WIDIH_16:  memory address is 16 bit 
  * @param  address: memory device address.
  * @param  mem_address: memory address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_memory_write_dma(i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_DMA_MA_TX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* disable dma request */
  i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_TX, FALSE);

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* transfer config */
  i2c_transmit_set(hi2c->i2cx, address, mem_address_width, I2C_RELOAD_MODE, I2C_GEN_START_WRITE);

  /* wait for the tdis flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDIS_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_2;
  }

  /* send memory address */
  if(i2c_memory_address_send(hi2c, mem_address_width, mem_address, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_3;
  }

  /* wait for the tcrld flag to be set */
  if (i2c_wait_flag(hi2c, I2C_TCRLD_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_4;
  }

  /* start transfer */
  i2c_start_transfer_dma(hi2c, hi2c->dma_tx_channel, address, I2C_WITHOUT_START);

  /* enable i2c interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_ACKFIAL_INT, TRUE);

  /* enable dma request */
  i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_TX, TRUE);

  return I2C_OK;
}

/**
  * @brief  read data from memory device through polling mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  mem_address_width: memory address width.
  *         this parameter can be one of the following values:
  *         - I2C_MEM_ADDR_WIDIH_8:  memory address is 8 bit 
  *         - I2C_MEM_ADDR_WIDIH_16:  memory address is 16 bit 
  * @param  address: memory device address.
  * @param  mem_address: memory address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_memory_read_dma(i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_DMA_MA_RX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* start transfer */
  i2c_transmit_set(hi2c->i2cx, address, mem_address_width, I2C_SOFT_STOP_MODE, I2C_GEN_START_WRITE);

  /* wait for the tdis flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDIS_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_2;
  }

  /* send memory address */
  if(i2c_memory_address_send(hi2c, mem_address_width, mem_address, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_3;
  }

  /* wait for the tdc flag to be set */
  if (i2c_wait_flag(hi2c, I2C_TDC_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_4;
  }

  /* disable dma request */
  i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_RX, FALSE);

  /* start transfer */
  i2c_start_transfer_dma(hi2c, hi2c->dma_rx_channel, address, I2C_GEN_START_READ);

  /* enable i2c interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_ACKFIAL_INT, TRUE);

  /* enable dma request */
  i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_RX, TRUE);

  return I2C_OK;
}

/**
  * @brief  the master transmits data through SMBus mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_smbus_master_transmit(i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->pbuff = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* enable pec calculation */
  i2c_pec_calculate_enable(hi2c->i2cx, TRUE);

  /* enable pec transmit request */
  i2c_pec_transmit_enable(hi2c->i2cx, TRUE);

  /* start transfer */
  i2c_start_transfer(hi2c, address, I2C_GEN_START_WRITE);

  hi2c->pcount--;

  while (hi2c->pcount > 0)
  {
    /* wait for the tdis flag to be set */
    if(i2c_wait_flag(hi2c, I2C_TDIS_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
      return I2C_ERR_STEP_2;
    }

    /* send data */
    i2c_data_send(hi2c->i2cx, *hi2c->pbuff++);
    hi2c->psize--;
    hi2c->pcount--;

    if ((hi2c->psize == 0) && (hi2c->pcount != 0))
    {
      /* wait for the tcrld flag to be set  */
      if (i2c_wait_flag(hi2c, I2C_TCRLD_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
      {
        return I2C_ERR_STEP_3;
      }

      /* continue transfer */
      i2c_start_transfer(hi2c, address, I2C_WITHOUT_START);
    }
  }

  /* wait for the stop flag to be set  */
  if(i2c_wait_flag(hi2c, I2C_STOPF_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_4;
  }

  /* clear stop flag */
  i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

  /* reset ctrl2 register */
  i2c_reset_ctrl2_register(hi2c);

  return I2C_OK;
}

/**
  * @brief  the slave receive data through SMBus mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_smbus_slave_receive(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->pbuff = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* enable pec calculation */
  i2c_pec_calculate_enable(hi2c->i2cx, TRUE);

  /* enable slave data control mode */
  i2c_slave_data_ctrl_enable(hi2c->i2cx, TRUE);

  /* enable acknowledge */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* wait for the addr flag to be set */
  if (i2c_wait_flag(hi2c, I2C_ADDRF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* enable pec transmit request */
  i2c_pec_transmit_enable(hi2c->i2cx, TRUE);

  /* configure the number of bytes to be transmitted */
  i2c_cnt_set(hi2c->i2cx, hi2c->pcount);

  /* clear addr flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ADDRF_FLAG);

  while (hi2c->pcount > 0)
  {
    /* wait for the rdbf flag to be set  */
    if(i2c_wait_flag(hi2c, I2C_RDBF_FLAG, I2C_EVENT_CHECK_STOP, timeout) != I2C_OK)
    {
      /* disable acknowledge */
      i2c_ack_enable(hi2c->i2cx, FALSE);

      /* if data is received, read data */
      if (i2c_flag_get(hi2c->i2cx, I2C_RDBF_FLAG) == SET)
      {
        /* read data */
        (*hi2c->pbuff++) = i2c_data_receive(hi2c->i2cx);
        hi2c->pcount--;
      }

      return I2C_ERR_STEP_3;
    }

    /* read data */
    (*hi2c->pbuff++) = i2c_data_receive(hi2c->i2cx);
    hi2c->pcount--;
  }

  /* wait for the stop flag to be set */
  if(i2c_wait_flag(hi2c, I2C_STOPF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    /* disable acknowledge */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    return I2C_ERR_STEP_4;
  }

  /* clear stop flag */
  i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    /* disable acknowledge */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    return I2C_ERR_STEP_5;
  }

  /* disable slave data control mode */
  i2c_slave_data_ctrl_enable(hi2c->i2cx, FALSE);

  return I2C_OK;
}

/**
  * @brief  the master receive data through SMBus mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_smbus_master_receive(i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->pbuff = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* enable pec calculation */
  i2c_pec_calculate_enable(hi2c->i2cx, TRUE);

  /* enable pec transmit request */
  i2c_pec_transmit_enable(hi2c->i2cx, TRUE);

  /* start transfer */
  i2c_start_transfer(hi2c, address, I2C_GEN_START_READ);

  while (hi2c->pcount > 0)
  {
    /* wait for the rdbf flag to be set  */
    if(i2c_wait_flag(hi2c, I2C_RDBF_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
      return I2C_ERR_STEP_2;
    }

    /* read data */
    (*hi2c->pbuff++) = i2c_data_receive(hi2c->i2cx);
    hi2c->pcount--;
    hi2c->psize--;

    if ((hi2c->psize == 0) && (hi2c->pcount != 0))
    {
      /* wait for the tcrld flag to be set  */
      if (i2c_wait_flag(hi2c, I2C_TCRLD_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
      {
        return I2C_ERR_STEP_3;
      }

      /* continue transfer */
      i2c_start_transfer(hi2c, address, I2C_WITHOUT_START);
    }
  }

  /* wait for the stop flag to be set  */
  if(i2c_wait_flag(hi2c, I2C_STOPF_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_4;
  }

  /* clear stop flag */
  i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

  /* reset ctrl2 register */
  i2c_reset_ctrl2_register(hi2c);

  return I2C_OK;
}

/**
  * @brief  the slave transmits data through SMBus mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_smbus_slave_transmit(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->pbuff = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* enable pec calculation */
  i2c_pec_calculate_enable(hi2c->i2cx, TRUE);

  /* enable slave data control mode */
  i2c_slave_data_ctrl_enable(hi2c->i2cx, TRUE);

  /* enable acknowledge */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* wait for the addr flag to be set */
  if (i2c_wait_flag(hi2c, I2C_ADDRF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    /* disable acknowledge */
    i2c_ack_enable(hi2c->i2cx, FALSE);
    return I2C_ERR_STEP_1;
  }

  /* if 7-bit address mode is selected */
  if (hi2c->i2cx->ctrl2_bit.addr10 == 0)
  {
    /* enable pec transmit request */
    i2c_pec_transmit_enable(hi2c->i2cx, TRUE);

    /* configure the number of bytes to be transmitted */
    i2c_cnt_set(hi2c->i2cx, hi2c->pcount);
  }

  /* clear addr flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ADDRF_FLAG);

  /* if 10-bit address mode is used */
  if (hi2c->i2cx->ctrl2_bit.addr10 != RESET)
  {
    /* wait for the addr flag to be set */
    if (i2c_wait_flag(hi2c, I2C_ADDRF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
    {
      /* disable acknowledge */
      i2c_ack_enable(hi2c->i2cx, FALSE);

      return I2C_ERR_STEP_2;
    }

    /* enable pec transmit request */
    i2c_pec_transmit_enable(hi2c->i2cx, TRUE);

    /* configure the number of bytes to be transmitted */
    i2c_cnt_set(hi2c->i2cx, hi2c->pcount);

    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDRF_FLAG);
  }

  hi2c->pcount--;

  while (hi2c->pcount > 0)
  {
    /* wait for the tdis flag to be set */
    if(i2c_wait_flag(hi2c, I2C_TDIS_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
      /* disable acknowledge */
      i2c_ack_enable(hi2c->i2cx, FALSE);

      return I2C_ERR_STEP_4;
    }

    /* send data */
    i2c_data_send(hi2c->i2cx, *hi2c->pbuff++);
    hi2c->pcount--;
  }

  /* wait for the ackfail flag to be set */
  if(i2c_wait_flag(hi2c, I2C_ACKFAIL_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_5;
  }

  /* clear ack fail flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);

  /* wait for the stop flag to be set */
  if(i2c_wait_flag(hi2c, I2C_STOPF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    /* disable acknowledge */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    return I2C_ERR_STEP_6;
  }

  /* clear stop flag */
  i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    /* disable acknowledge */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    return I2C_ERR_STEP_7;
  }

  /* reset ctrl2 register */
  i2c_reset_ctrl2_register(hi2c);

  /* refresh tx dt register */
  i2c_refresh_txdt_register(hi2c);

  /* disable slave data control mode */
  i2c_slave_data_ctrl_enable(hi2c->i2cx, FALSE);

  return I2C_OK;
}

/**
  * @brief  master interrupt processing function in interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @retval i2c status.
  */
i2c_status_type i2c_master_irq_handler_int(i2c_handle_type* hi2c)
{
  if (i2c_flag_get(hi2c->i2cx, I2C_ACKFAIL_FLAG) != RESET)
  {
    /* clear ackfail flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);

    /* refresh tx register */
    i2c_refresh_txdt_register(hi2c);

    if(hi2c->pcount != 0)
    {
      hi2c->error_code = I2C_ERR_ACKFAIL;
    }
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_TDIS_FLAG) != RESET)
  {
    /* send data */
    i2c_data_send(hi2c->i2cx, *hi2c->pbuff++);
    hi2c->pcount--;
    hi2c->psize--;
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_TCRLD_FLAG) != RESET)
  {
    if ((hi2c->psize == 0) && (hi2c->pcount != 0))
    {
      /* continue transfer */
      i2c_start_transfer(hi2c, i2c_transfer_addr_get(hi2c->i2cx), I2C_WITHOUT_START);
    }
    else
    {
      return I2C_ERR_TCRLD;
    }
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_RDBF_FLAG) != RESET)
  {
    /* read data */
    (*hi2c->pbuff++) = i2c_data_receive(hi2c->i2cx);
    hi2c->pcount--;
    hi2c->psize--;
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_TDC_FLAG) != RESET)
  {
    if (hi2c->pcount == 0)
    {
      if (hi2c->i2cx->ctrl2_bit.astopen == 0)
      {
        /* generate stop condtion */
        i2c_stop_generate(hi2c->i2cx);
      }
    }
    else
    {
      return I2C_ERR_TDC;
    }
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_STOPF_FLAG) != RESET)
  {
    /* clear stop flag */
    i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

    /* reset ctrl2 register */
    i2c_reset_ctrl2_register(hi2c);

    if (i2c_flag_get(hi2c->i2cx, I2C_ACKFAIL_FLAG) != RESET)
    {
      /* clear ackfail flag */
      i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);
    }

    /* refresh tx dt register */
    i2c_refresh_txdt_register(hi2c);

    /* disable interrupts */
    i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_TDC_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_TD_INT | I2C_RD_INT, FALSE);

    /* transfer complete */
    hi2c->status = I2C_END;
  }

  return I2C_OK;
}

/**
  * @brief  slave interrupt processing function in interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @retval i2c status.
  */
i2c_status_type i2c_slave_irq_handler_int(i2c_handle_type* hi2c)
{
  if (i2c_flag_get(hi2c->i2cx, I2C_ACKFAIL_FLAG) != RESET)
  {
    /* transfer complete */
    if (hi2c->pcount == 0)
    {
      i2c_refresh_txdt_register(hi2c);

      /* clear ackfail flag */
      i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);
    }
    /* the transfer has not been completed */
    else
    {
      /* clear ackfail flag */
      i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);
    }
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_ADDRF_FLAG) != RESET)
  {
    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDRF_FLAG);
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_TDIS_FLAG) != RESET)
  {
    if (hi2c->pcount > 0)
    {
      /* send data */
      hi2c->i2cx->txdt = (*(hi2c->pbuff++));
      hi2c->psize--;
      hi2c->pcount--;
    }
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_RDBF_FLAG) != RESET)
  {
    if (hi2c->pcount > 0)
    {
      /* read data */
      (*hi2c->pbuff++) = i2c_data_receive(hi2c->i2cx);
      hi2c->pcount--;
      hi2c->psize--;
    }
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_STOPF_FLAG) != RESET)
  {
    /* clear stop flag */
    i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

    /* disable interrupts */
    i2c_interrupt_enable(hi2c->i2cx, I2C_ADDR_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_ERR_INT | I2C_TDC_INT | I2C_TD_INT | I2C_RD_INT, FALSE);

    /* reset ctrl2 register */
    i2c_reset_ctrl2_register(hi2c);

    /* refresh tx dt register */
    i2c_refresh_txdt_register(hi2c);

    /* if data is received, read data */
    if (i2c_flag_get(hi2c->i2cx, I2C_RDBF_FLAG) != RESET)
    {
      /* read data */
      (*hi2c->pbuff++) = i2c_data_receive(hi2c->i2cx);

      if ((hi2c->psize > 0))
      {
        hi2c->pcount--;
        hi2c->psize--;
      }
    }

    /* transfer complete */
    hi2c->status = I2C_END;
  }

  return I2C_OK;
}

/**
  * @brief  master interrupt processing function in dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @retval i2c status.
  */
i2c_status_type i2c_master_irq_handler_dma(i2c_handle_type* hi2c)
{
  if (i2c_flag_get(hi2c->i2cx, I2C_ACKFAIL_FLAG) != RESET)
  {
    /* clear ackfail flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);

    /* enable stop interrupt to wait for stop generate stop */
    i2c_interrupt_enable(hi2c->i2cx, I2C_STOP_INT, TRUE);

    /* refresh tx dt register */
    i2c_refresh_txdt_register(hi2c);

    if(hi2c->pcount != 0)
    {
      hi2c->error_code = I2C_ERR_ACKFAIL;
    }
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_TCRLD_FLAG) != RESET)
  {
    /* disable tdc interrupt */
    i2c_interrupt_enable(hi2c->i2cx, I2C_TDC_INT, FALSE);

    if (hi2c->pcount != 0)
    {
      /* continue transfer */
      i2c_start_transfer(hi2c, i2c_transfer_addr_get(hi2c->i2cx), I2C_WITHOUT_START);

      /* enable dma request */
      if (hi2c->dma_init_struct.direction == DMA_DIR_MEMORY_TO_PERIPHERAL)
      {
        i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_TX, TRUE);
      }
      else
      {
        i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_RX, TRUE);
      }
    }
    else
    {
      return I2C_ERR_TCRLD;
    }
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_STOPF_FLAG) != RESET)
  {
    /* clear stop flag */
    i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

    /* reset ctrl2 register */
    i2c_reset_ctrl2_register(hi2c);

    if (i2c_flag_get(hi2c->i2cx, I2C_ACKFAIL_FLAG) != RESET)
    {
      /* clear ackfail flag */
      i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);
    }

    /* refresh tx dt register */
    i2c_refresh_txdt_register(hi2c);

    /* disable interrupts */
    i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_TDC_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_TD_INT | I2C_RD_INT, FALSE);

    /* transfer complete */
    hi2c->status = I2C_END;
  }

  return I2C_OK;
}

/**
  * @brief  slave interrupt processing function in dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @retval i2c status.
  */
i2c_status_type i2c_slave_irq_handler_dma(i2c_handle_type* hi2c)
{
  if (i2c_flag_get(hi2c->i2cx, I2C_ACKFAIL_FLAG) != RESET)
  {
    /* clear ackfail flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_ADDRF_FLAG) != RESET)
  {
    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDRF_FLAG);
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_STOPF_FLAG) != RESET)
  {
    /* clear stop flag */
    i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

    /* disable interrupts */
    i2c_interrupt_enable(hi2c->i2cx, I2C_ADDR_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_ERR_INT | I2C_TDC_INT | I2C_TD_INT | I2C_RD_INT, FALSE);

    /* reset ctrl2 register */
    i2c_reset_ctrl2_register(hi2c);

    /* refresh tx dt register */
    i2c_refresh_txdt_register(hi2c);

    /* if data is received, read data */
    if (i2c_flag_get(hi2c->i2cx, I2C_RDBF_FLAG) != RESET)
    {
      /* read data */
      (*hi2c->pbuff++) = i2c_data_receive(hi2c->i2cx);

      if ((hi2c->psize > 0))
      {
        hi2c->pcount--;
        hi2c->psize--;
      }
    }

    /* transfer complete */
    hi2c->status = I2C_END;
  }

  return I2C_OK;
}

/**
  * @brief  dma processing function.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_dma_tx_rx_irq_handler(i2c_handle_type* hi2c, dma_channel_type* dma_channel)
{
  /* transfer complete */
  if (dma_flag_get(DMA_GET_TC_FLAG(dma_channel)) != RESET)
  {
    /* disable the transfer complete interrupt */
    dma_interrupt_enable(dma_channel, DMA_FDT_INT, FALSE);

    /* clear the transfer complete flag */
    dma_flag_clear(DMA_GET_TC_FLAG(dma_channel));

    /* disable dma request */
    i2c_dma_enable(hi2c->i2cx, DMA_GET_REQUEST(dma_channel), FALSE);

    /* disable dma channel */
    dma_channel_enable(dma_channel, FALSE);

    switch(hi2c->mode)
    {
      case I2C_DMA_MA_TX:
      case I2C_DMA_MA_RX:
      {
        /* update the number of transfers */
        hi2c->pcount -= hi2c->psize;

        /* transfer complete */
        if (hi2c->pcount == 0)
        {
          /* enable stop interrupt */
          i2c_interrupt_enable(hi2c->i2cx, I2C_STOP_INT, TRUE);
        }
        /* the transfer has not been completed */
        else
        {
          /* update the buffer pointer of transfers */
          hi2c->pbuff += hi2c->psize;

          /* set the number to be transferred */
          if (hi2c->pcount > MAX_TRANSFER_CNT)
          {
            hi2c->psize = MAX_TRANSFER_CNT;
          }
          else
          {
            hi2c->psize = hi2c->pcount;
          }

          /* config dma channel, continue to transfer data */
          i2c_dma_config(hi2c, dma_channel, hi2c->pbuff, hi2c->psize);

          /* enable tdc interrupt */
          i2c_interrupt_enable(hi2c->i2cx, I2C_TDC_INT, TRUE);
        }
      }break;
      case I2C_DMA_SLA_TX:
      case I2C_DMA_SLA_RX:
      {

      }break;

      default:break;
    }
  }
}

/**
  * @brief  dma transmission complete interrupt function.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_dma_tx_irq_handler(i2c_handle_type* hi2c)
{
  i2c_dma_tx_rx_irq_handler(hi2c, hi2c->dma_tx_channel);
}

/**
  * @brief  dma reveive complete interrupt function.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_dma_rx_irq_handler(i2c_handle_type* hi2c)
{
  i2c_dma_tx_rx_irq_handler(hi2c, hi2c->dma_rx_channel);
}

/**
  * @brief  interrupt procession function.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_evt_irq_handler(i2c_handle_type* hi2c)
{
  switch(hi2c->mode)
  {
    case I2C_INT_MA_TX:
    case I2C_INT_MA_RX:
    {
      i2c_master_irq_handler_int(hi2c);
    }break;
    case I2C_INT_SLA_TX:
    case I2C_INT_SLA_RX:
    {
      i2c_slave_irq_handler_int(hi2c);
    }break;
    case I2C_DMA_MA_TX:
    case I2C_DMA_MA_RX:
    {
      i2c_master_irq_handler_dma(hi2c);
    }break;
    case I2C_DMA_SLA_TX:
    case I2C_DMA_SLA_RX:
    {
      i2c_slave_irq_handler_dma(hi2c);
    }break;

    default:break;
  }
}

/**
  * @brief  dma reveive complete interrupt function.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_err_irq_handler(i2c_handle_type* hi2c)
{ 
  /* buserr */
  if (i2c_flag_get(hi2c->i2cx, I2C_BUSERR_FLAG) != RESET)
  {
    hi2c->error_code = I2C_ERR_INTERRUPT;
    
    /* clear flag */
    i2c_flag_clear(hi2c->i2cx, I2C_BUSERR_FLAG);
    
    /* disable interrupts */
    i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT, FALSE);
  }

  /* arlost */
  if (i2c_flag_get(hi2c->i2cx, I2C_ARLOST_FLAG) != RESET)
  {
    hi2c->error_code = I2C_ERR_INTERRUPT;
    
    /* clear flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ARLOST_FLAG);
    
    /* disable interrupts */
    i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT, FALSE);
  }

  /* ouf */
  if (i2c_flag_get(hi2c->i2cx, I2C_OUF_FLAG) != RESET)
  {
    hi2c->error_code = I2C_ERR_INTERRUPT;
    
    /* clear flag */
    i2c_flag_clear(hi2c->i2cx, I2C_OUF_FLAG);
    
    /* disable interrupts */
    i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT, FALSE);
  }

  /* pecerr */
  if (i2c_flag_get(hi2c->i2cx, I2C_PECERR_FLAG) != RESET)
  {
    hi2c->error_code = I2C_ERR_INTERRUPT;
    
    /* clear flag */
    i2c_flag_clear(hi2c->i2cx, I2C_PECERR_FLAG);
    
    /* disable interrupts */
    i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT, FALSE);

  }

  /* timeout */
  if (i2c_flag_get(hi2c->i2cx, I2C_TMOUT_FLAG) != RESET)
  {
    hi2c->error_code = I2C_ERR_INTERRUPT;
    
    /* clear flag */
    i2c_flag_clear(hi2c->i2cx, I2C_TMOUT_FLAG);
    
    /* disable interrupts */
    i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT, FALSE);
  }

  /* alertf */
  if (i2c_flag_get(hi2c->i2cx, I2C_ALERTF_FLAG) != RESET)
  {
    hi2c->error_code = I2C_ERR_INTERRUPT;
    
    /* clear flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ALERTF_FLAG);
    
    /* disable interrupts */
    i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT, FALSE);
  }
}


