/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifdef USBD_USE_CDC

#include "USBSerial.h"
#include "cdc_class.h"
#include "cdc_desc.h"
#include "usbd_int.h"

USBSerial SerialUSB;

usbd_core_type usb_core_dev;
uint8_t send_zero_packet = 0;
uint32_t timeout;

void USBSerial::begin(void)
{
  crm_usb_clock_source_select(CRM_USB_CLOCK_SOURCE_HICK);

  /* enable the acc calibration ready interrupt */
  crm_periph_clock_enable(CRM_ACC_PERIPH_CLOCK, TRUE);

  /* update the c1\c2\c3 value */
  acc_write_c1(7980);
  acc_write_c2(8000);
  acc_write_c3(8020);

  /* open acc calibration */
  acc_calibration_mode_enable(ACC_CAL_HICKTRIM, TRUE);

  /* enable usb clock */
  crm_periph_clock_enable(CRM_USB_PERIPH_CLOCK, TRUE);

  /* enable usb interrupt */
  nvic_irq_enable(USBFS_L_CAN1_RX0_IRQn, 0, 0);

  /* usb core init */
  usbd_core_init(&usb_core_dev, USB, &cdc_class_handler, &cdc_desc_handler, 0);

  /* enable usb pull-up */
  usbd_connect(&usb_core_dev);
}

void USBSerial::begin(uint32_t /* baud_count */)
{
  // uart config is ignored in USB-CDC
  begin();
}

void USBSerial::begin(uint32_t /* baud_count */, uint8_t /* config */)
{
  // uart config is ignored in USB-CDC
  begin();
}

void USBSerial::end()
{
  usbd_disconnect(&usb_core_dev);
}

int USBSerial::availableForWrite()
{
  // return available for write
  //return static_cast<int>(CDC_TransmitQueue_WriteSize(&TransmitQueue));
}

size_t USBSerial::write(uint8_t ch)
{
  // Just write single-byte buffer.
  return write(&ch, 1);
}

size_t USBSerial::write(const uint8_t *buffer, size_t size)
{

  /* send data to host */
  usb_vcp_send_data(&usb_core_dev, buffer, size);

  /* send data to host */
  usb_vcp_send_data(&usb_core_dev, buffer, 0);

  return 0;
}

int USBSerial::available(void)
{
  int data_len;
  // available for reading and return available data length
  if(usb_read_index == usb_rx_index)
  {
    data_len = 0;
  }
  else
  {
    if(usb_rx_index > usb_read_index)
      data_len = usb_rx_index - usb_read_index;
    else if(usb_rx_index == 0 && usb_rx_index != usb_read_index)
      data_len = usb_rx_buffer_size - usb_read_index;
    else
      data_len = (usb_rx_buffer_size - 1) + usb_rx_index - usb_read_index;
  }

  return data_len;
}

int USBSerial::read(void)
{
  // Read one byte from buffer.
  uint8_t ch;
  ch = usb_rx_buffer[usb_read_index];
  usb_read_index++;
  return ch;
}

size_t USBSerial::readBytes(char *buffer, size_t length)
{
  int data_len,index;
  if(usb_read_index == usb_rx_index)
  {
    //no data
    return 0;
  }
  else
  {
    if(usb_rx_index > usb_read_index)
      data_len = usb_rx_index - usb_read_index;
    else if(usb_rx_index == 0 && usb_rx_index != usb_read_index)
      data_len = usb_rx_buffer_size - usb_read_index;
    else
      data_len = (usb_rx_buffer_size - 1) + usb_rx_index - usb_read_index;
  }

  if(data_len < length)
  {
    if((usb_read_index + data_len) < usb_rx_buffer_size)
    {
      for(index = 0; index < data_len; index++)
      {
        *buffer++ = usb_rx_buffer[usb_read_index];
        usb_read_index++;
      }      
    }
    else
    {
      for(index = 0; index < data_len; index++)
      {
        *buffer++ = usb_rx_buffer[usb_read_index];
        usb_read_index++;
        if(usb_read_index == usb_rx_buffer_size)
          usb_read_index = 0;
      } 
    }
    return data_len;

  }
  else
  {
    if((usb_read_index + length) < usb_rx_buffer_size)
    {
      for(index = 0; index < length; index++)
      {
        *buffer++ = usb_rx_buffer[usb_read_index];
        usb_read_index++;
      }      
    }
    else
    {
      for(index = 0; index < length; index++)
      {
        *buffer++ = usb_rx_buffer[usb_read_index];
        usb_read_index++;
        if(usb_read_index == usb_rx_buffer_size)
          usb_read_index = 0;
      } 
    }
  }
  return length;
}

size_t USBSerial::readBytesUntil(char terminator, char *buffer, size_t length)
{
  
}

int USBSerial::peek(void)
{
  // Peek one symbol, it can't change receive avaiablity
 // return CDC_ReceiveQueue_Peek(&ReceiveQueue);
}

void USBSerial::flush(void)
{
  
}

uint32_t USBSerial::baud()
{
  return 115200;
}

uint8_t USBSerial::stopbits()
{
  return ONE_STOP_BIT;
}

uint8_t USBSerial::paritytype()
{
  return NO_PARITY;
}

uint8_t USBSerial::numbits()
{
  return 8;
}

void USBSerial::dtr(bool enable)
{
  //CDC_enableDTR(enable);
}

bool USBSerial::dtr(void)
{
  //return dtrState;
}

bool USBSerial::rts(void)
{
  //return rtsState;
}

USBSerial::operator bool()
{
  //delay(10);
  //return dtrState;
}


extern "C" void USBFS_L_CAN1_RX0_IRQHandler(void)
{
  usbd_irq_handler(&usb_core_dev);
}


#endif //USBD_USE_CDC
