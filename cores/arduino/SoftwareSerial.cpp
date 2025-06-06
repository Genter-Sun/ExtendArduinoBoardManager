/*
 * SoftwareSerial.cpp (formerly NewSoftSerial.cpp)
 *
 * Multi-instance software serial library for Arduino/Wiring
 * -- Interrupt-driven receive and other improvements by ladyada
 *    (http://ladyada.net)
 * -- Tuning, circular buffer, derivation from class Print/Stream,
 *    multi-instance support, porting to 8MHz processors,
 *    various optimizations, PROGMEM delay tables, inverse logic and
 *    direct port writing by Mikal Hart (http://www.arduiniana.org)
 * -- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
 * -- 20MHz processor support by Garrett Mace (http://www.macetech.com)
 * -- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)
 * -- STM32 support by Armin van der Togt
 * -- AT32F4 support by WeactStudio
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * The latest version of this library can always be found at
 * http://arduiniana.org.
 */

//
// Includes
//
#include "SoftwareSerial.h"
#include "timer.h"
#define OVERSAMPLE 3 // in RX, Timer will generate interruption OVERSAMPLE time during a bit. Thus OVERSAMPLE ticks in a bit. (interrupt not synchronized with edge).

// defined in bit-periods
#define HALFDUPLEX_SWITCH_DELAY 5
// It's best to define TIMER_SERIAL in variant.h. If not defined, we choose one here
// The order is based on (lack of) features and compare channels, we choose the simplest available
// because we only need an update interrupt

#if !defined(TIMER_SERIAL)
  #if defined (TMR18_BASE)
    #define TIMER_SERIAL TIM18
  #elif defined (TMR7_BASE)
    #define TIMER_SERIAL TIM7
  #elif defined (TMR6_BASE)
    #define TIMER_SERIAL TIM6
  #elif defined (TMR22_BASE)
    #define TIMER_SERIAL TIM22
  #elif defined (TMR21_BASE)
    #define TIMER_SERIAL TIM21
  #elif defined (TMR17_BASE)
    #define TIMER_SERIAL TIM17
  #elif defined (TMR16_BASE)
    #define TIMER_SERIAL TIM16
  #elif defined (TMR15_BASE)
    #define TIMER_SERIAL TIM15
  #elif defined (TMR14_BASE)
    #define TIMER_SERIAL TIM14
  #elif defined (TMR13_BASE)
    #define TIMER_SERIAL TIM13
  #elif defined (TMR11_BASE)
    #define TIMER_SERIAL TIM11
  #elif defined (TMR10_BASE)
    #define TIMER_SERIAL TIM10
  #elif defined (TMR12_BASE)
    #define TIMER_SERIAL TIM12
  #elif defined (TMR19_BASE)
    #define TIMER_SERIAL TIM19
  #elif defined (TMR9_BASE)
    #define TIMER_SERIAL TIM9
  #elif defined (TMR5_BASE)
    #define TIMER_SERIAL TIM5
  #elif defined (TMR4_BASE)
    #define TIMER_SERIAL TIM4
  #elif defined (TMR3_BASE)
    #define TIMER_SERIAL TIM3
  #elif defined (TMR2_BASE)
    #define TIMER_SERIAL TIM2
  #elif defined (TMR20_BASE)
    #define TIMER_SERIAL TIM20
  #elif defined (TMR8_BASE)
    #define TIMER_SERIAL TIM8
  #elif defined (TMR1_BASE)
    #define TIMER_SERIAL TIM1
  #else
    #error No suitable timer found for SoftwareSerial, define TIMER_SERIAL in variant.h
  #endif
#endif
//
// Static
//
SoftwareSerial *SoftwareSerial::active_listener = nullptr;
SoftwareSerial *volatile SoftwareSerial::active_out = nullptr;
SoftwareSerial *volatile SoftwareSerial::active_in = nullptr;
int32_t SoftwareSerial::tx_tick_cnt = 0; // OVERSAMPLE ticks needed for a bit
int32_t volatile SoftwareSerial::rx_tick_cnt = 0;  // OVERSAMPLE ticks needed for a bit
uint32_t SoftwareSerial::tx_buffer = 0;
int32_t SoftwareSerial::tx_bit_cnt = 0;
uint32_t SoftwareSerial::rx_buffer = 0;
int32_t SoftwareSerial::rx_bit_cnt = -1; // rx_bit_cnt = -1 :  waiting for start bit
uint32_t SoftwareSerial::cur_speed = 0;

//
// Private methods
//

void SoftwareSerial::setSpeed(uint32_t speed)
{
  if (speed != cur_speed) {
    // Timer_Pause(TIMER_SERIAL);
    if (speed != 0) {
      // Disable the timer
      uint32_t clock_rate, cmp_value;
      // Get timer clock
      clock_rate = Timer_GetClockMax(TIMER_SERIAL);
      int pre = 1;
      // Calculate prescale an compare value
      do {
        cmp_value = clock_rate / (speed * OVERSAMPLE);
        if (cmp_value >= UINT16_MAX) {
          clock_rate = clock_rate / 2;
          pre *= 2;
        }
      } while (cmp_value >= UINT16_MAX);
      Timer_SetPrescaler(TIMER_SERIAL, pre - 1);
      Timer_SetReload(TIMER_SERIAL, cmp_value - 1);
      Timer_SetCounter(TIMER_SERIAL, 0);
      // Timer_Resume(TIMER_SERIAL);
    } else {

    }
    cur_speed = speed;
  }
}

// This function sets the current object as the "listening"
// one and returns true if it replaces another
bool SoftwareSerial::listen()
{
  if (active_listener != this) {
    // wait for any transmit to complete as we may change speed
    while (active_out);
    if (active_listener != nullptr) {
      active_listener->stopListening();
    }
    rx_tick_cnt = 1; // 1 : next interrupt will decrease rx_tick_cnt to 0 which means RX pin level will be considered.
    rx_bit_cnt = -1; // rx_bit_cnt = -1 :  waiting for start bit
    setSpeed(_speed);
    active_listener = this;
    if (!_half_duplex) {
      active_in = this;
    } else if (!active_out) {
      setRXTX(true);
    }
    return true;
  }
  return false;
}

// Stop listening. Returns true if we were actually listening.
bool SoftwareSerial::stopListening()
{
  if (active_listener == this) {
    // wait for any output to complete
    while (active_out);
    if (_half_duplex) {
      setRXTX(false);
    }
    active_listener = nullptr;
    active_in = nullptr;
    // turn off ints
    setSpeed(0);
    return true;
  }
  return false;
}

inline void SoftwareSerial::setTX()
{
  if (_inverse_logic) {
    GPIO_LOW(_transmitPinPort, _transmitPinNumber);
  } else {
    GPIO_HIGH(_transmitPinPort, _transmitPinNumber);
  }
  pinMode(_transmitPin, OUTPUT);
}

inline void SoftwareSerial::setRX()
{
  pinMode(_receivePin, _inverse_logic ? INPUT_PULLDOWN : INPUT_PULLUP); // pullup for normal logic!
}

inline void SoftwareSerial::setRXTX(bool input)
{
  if (_half_duplex) {
    if (input) {
      if (active_in != this) {
        setRX();
        rx_bit_cnt = -1; // rx_bit_cnt = -1 :  waiting for start bit
        rx_tick_cnt = 2; // 2 : next interrupt will be discarded. 2 interrupts required to consider RX pin level
        active_in = this;
      }
    } else {
      if (active_in == this) {
        setTX();
        active_in = nullptr;
      }
    }
  }
}

inline void SoftwareSerial::send()
{
  if (--tx_tick_cnt <= 0) { // if tx_tick_cnt > 0 interrupt is discarded. Only when tx_tick_cnt reach 0 we set TX pin.
    if (tx_bit_cnt++ < 10) { // tx_bit_cnt < 10 transmission is not fiisehed (10 = 1 start +8 bits + 1 stop)
      // send data (including start and stop bits)
      if (tx_buffer & 1) {
        GPIO_HIGH(_transmitPinPort, _transmitPinNumber);
      } else {
        GPIO_LOW(_transmitPinPort, _transmitPinNumber);
      }
      tx_buffer >>= 1;
      tx_tick_cnt = OVERSAMPLE; // Wait OVERSAMPLE tick to send next bit
    } else { // Transmission finished
      tx_tick_cnt = 1;
      if (_output_pending) {
        active_out = nullptr;

        // When in half-duplex mode, wait for HALFDUPLEX_SWITCH_DELAY bit-periods after the byte has
        // been transmitted before allowing the switch to RX mode
      } else if (tx_bit_cnt > 10 + OVERSAMPLE * HALFDUPLEX_SWITCH_DELAY) {
        if (_half_duplex && active_listener == this) {
          setRXTX(true);
        }
        active_out = nullptr;
      }
    }
  }
}

//
// The receive routine called by the interrupt handler
//
inline void SoftwareSerial::recv()
{
  if (--rx_tick_cnt <= 0) { // if rx_tick_cnt > 0 interrupt is discarded. Only when rx_tick_cnt reach 0 RX pin is considered
    bool inbit = GPIO_READ(_receivePinPort, _receivePinNumber) ^ _inverse_logic;
    if (rx_bit_cnt == -1) {  // rx_bit_cnt = -1 :  waiting for start bit
      if (!inbit) {
        // got start bit
        rx_bit_cnt = 0; // rx_bit_cnt == 0 : start bit received
        rx_tick_cnt = OVERSAMPLE + 1; // Wait 1 bit (OVERSAMPLE ticks) + 1 tick in order to sample RX pin in the middle of the edge (and not too close to the edge)
        rx_buffer = 0;
      } else {
        rx_tick_cnt = 1; // Waiting for start bit, but we don't get right level. Wait for next Interrupt to ckech RX pin level
      }
    } else if (rx_bit_cnt >= 8) { // rx_bit_cnt >= 8 : waiting for stop bit
      if (inbit) {
        // stop bit read complete add to buffer
        uint8_t next = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
        if (next != _receive_buffer_head) {
          // save new data in buffer: tail points to where byte goes
          _receive_buffer[_receive_buffer_tail] = rx_buffer; // save new byte
          _receive_buffer_tail = next;
        } else { // rx_bit_cnt = x  with x = [0..7] correspond to new bit x received
          _buffer_overflow = true;
        }
      }
      // Full trame received. Restart waiting for start bit at next interrupt
      rx_tick_cnt = 1;
      rx_bit_cnt = -1;
    } else {
      // data bits
      rx_buffer >>= 1;
      if (inbit) {
        rx_buffer |= 0x80;
      }
      rx_bit_cnt++; // Preprare for next bit
      rx_tick_cnt = OVERSAMPLE; // Wait OVERSAMPLE ticks before sampling next bit
    }
  }
}

//
// Interrupt handling
//

/* static */
inline void SoftwareSerial::handleInterrupt()
{
  if (active_in) {
    active_in->recv();
  }
  if (active_out) {
    active_out->send();
  }
}
//
// Constructor
//
SoftwareSerial::SoftwareSerial(uint16_t receivePin, uint16_t transmitPin, bool inverse_logic /* = false */) :
  _receivePin(receivePin),
  _transmitPin(transmitPin),
  _receivePinPort(digitalPinToPort(receivePin)),
  _receivePinNumber(digitalPinToBitMask(receivePin)),
  _transmitPinPort(digitalPinToPort(transmitPin)),
  _transmitPinNumber(digitalPinToBitMask(transmitPin)),
  _speed(0),
  _buffer_overflow(false),
  _inverse_logic(inverse_logic),
  _half_duplex(receivePin == transmitPin),
  _output_pending(0),
  _receive_buffer_tail(0),
  _receive_buffer_head(0)
{

}

//
// Destructor
//
SoftwareSerial::~SoftwareSerial()
{
  end();
}

//
// Public methods
//

void SoftwareSerial::begin(long speed)
{
#ifdef FORCE_BAUD_RATE
  speed = FORCE_BAUD_RATE;
#endif
  _speed = speed;
  Timer_SetInterrupt(TIMER_SERIAL, 1000, handleInterrupt);
  if (!_half_duplex) {
    setTX();
    setRX();
    listen();
  } else {
    setTX();
  }
}

void SoftwareSerial::end()
{
  stopListening();
}

// Read data from buffer
int SoftwareSerial::read()
{
  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail) {
    return -1;
  }

  // Read from "head"
  uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
  return d;
}

int SoftwareSerial::available()
{
  return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

size_t SoftwareSerial::write(uint8_t b)
{
  // wait for previous transmit to complete
  _output_pending = 1;
  while (active_out)
    ;
  // add start and stop bits.
  tx_buffer = b << 1 | 0x200;
  if (_inverse_logic) {
    tx_buffer = ~tx_buffer;
  }
  tx_bit_cnt = 0;
  tx_tick_cnt = OVERSAMPLE;
  setSpeed(_speed);
  if (_half_duplex) {
    setRXTX(false);
  }
  _output_pending = 0;
  // make us active
  active_out = this;
  return 1;
}

void SoftwareSerial::flush()
{
  noInterrupts();
  _receive_buffer_head = _receive_buffer_tail = 0;
  interrupts();
}

int SoftwareSerial::peek()
{
  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail) {
    return -1;
  }

  // Read from "head"
  return _receive_buffer[_receive_buffer_head];
}

void SoftwareSerial::setInterruptPriority(uint32_t preemptPriority, uint32_t subPriority)
{

}
