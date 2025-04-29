/*
  Mouse.cpp

  Copyright (c) 2015, Arduino LLC
  Original code (pre-library): Copyright (c) 2011, Peter Barrett

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifdef USBD_USE_MOUSE
#include "Mouse.h"

#include "mouse_class.h"
#include "mouse_desc.h"
#include "usbd_int.h"

Mouse_ Mouse;

usbd_core_type usb_core_dev;
//================================================================================
//================================================================================
//  Mouse

Mouse_::Mouse_(void) : _buttons(0)
{
}

void Mouse_::begin(void)
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
  usbd_core_init(&usb_core_dev, USB, &mouse_class_handler, &mouse_desc_handler, 0);

  /* enable usb pull-up */
  usbd_connect(&usb_core_dev);
}

void Mouse_::end(void)
{
  usbd_disconnect(&usb_core_dev);
}

void Mouse_::click(uint8_t b)
{
  _buttons = b;
  move(0, 0, 0);
  _buttons = 0;
  move(0, 0, 0);
}

void Mouse_::move(signed char x, signed char y, signed char wheel)
{
  uint8_t m[4];
  m[0] = _buttons;
  m[1] = x;
  m[2] = y;
  m[3] = wheel;

  if(usbd_connect_state_get(&usb_core_dev) == USB_CONN_STATE_CONFIGURED)
  {
    usb_mouse_class_send_report(&usb_core_dev, m, 4);
  }
}

void Mouse_::buttons(uint8_t b)
{
  if (b != _buttons) {
    _buttons = b;
    move(0, 0, 0);
  }
}

void Mouse_::press(uint8_t b)
{
  buttons(_buttons | b);
}

void Mouse_::release(uint8_t b)
{
  buttons(_buttons & ~b);
}

bool Mouse_::isPressed(uint8_t b)
{
  if ((b & _buttons) > 0) {
    return true;
  }
  return false;
}

extern "C" void USBFS_L_CAN1_RX0_IRQHandler(void)
{
  usbd_irq_handler(&usb_core_dev);
}

#endif
