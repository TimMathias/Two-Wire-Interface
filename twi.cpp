//
// Asynchronous Two-Wire Interface (IIC or I2C) for ATmega48A/PA/88A/PA/168A/PA/328/P
//
// Implements non-blocking (asynchronous) burst-write and burst-read using the TWI ISR and WDT ISR.
//
// The non-blocking Read/Write functions initiate the transaction, then return to the caller,
// then the ISR continues the transaction to completion and sets flags to indicate completion and result status.
//
// Alternatively, the Read/Write functions can wait (i.e. block) until completion by specifying the Wait mode.
//
// Clock frequency conforms to the SMBus specification when the _smbus_mode flag is true.
//
// Timeouts partially conform to the SMBus specification, i.e. a fixed 32 ms timeout.
//
// Updated the terms "master/slave" to "controller/target" throughout to align with MIPI I3C
// specification and NXP's Inclusive Language Project.
// UM10204, I2C-bus specification and user manual, Rev. 7.0 — 1 October 2021:
// https://www.nxp.com/webapp/Download?colCode=UM10204
// https://www.nxp.com/docs/en/application-note/AN4471.pdf
// https://www.nxp.com/docs/en/user-guide/UM10204.pdf
//
// ATmega328p datasheet:
// https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061A.pdf
//
// SMBus specification:
// http://smbus.org/specs/SMBus_3_1_20180319.pdf
//
//
// MIT License
//
// Copyright (c) 2021-2022 Timothy Mathias
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "twi.h"

TWI::Sequences TWI::_sequence;
volatile TWI::States TWI::_state = States::Ready;
volatile TWI::Results TWI::_result = Results::Unknown;

// Internal pullup resistors on SCL and SDA
//   true : enable them.
//   false: disable them.
volatile bool TWI::_use_pullups = false;

// TWI ISR
//   true : Asynchronous (non-blocking) transactions using the TWI ISR.
//   false: Synchronous (blocking) transactions by polling the TWINT flag.
volatile bool TWI::_use_isr = true;

bool TWI::_smbus_mode = false;
volatile byte* TWI::_buffer;               // Pointer to caller's buffer.
volatile uint16_t TWI::_count;             // Number of bytes to transfer.
volatile uint16_t TWI::_index;             // Index into caller's buffer.
volatile byte TWI::_target_address_read;   // Composite 7-bit target address merged with read bit (1).
volatile byte TWI::_target_address_write;  // Composite 7-bit target address merged with write bit (0).
volatile uint32_t TWI::_internal_address;  // Internal register to read/write.
volatile byte TWI::_internal_address_size; // Size of internal register address: 1 to 3 bytes. 0 means there is no internal address.
volatile byte TWI::_data_byte;             // data_byte to read/write.
volatile bool TWI::_timeout;               // Flag to indicate timeout condition.

void TWI::Enable(const uint32_t twi_freq = 400000u, const bool use_pullups = false)//, const uint16_t twi_mext_timeout = 10000u)
{
  // Disable TWI ACK, TWI module and TWI interrupt.
  TWCR &= ~((1 << TWEA) | (1 << TWEN) | (1 << TWIE));

  _state = States::Ready;
  _result = Results::Unknown;
  _use_pullups = use_pullups;

  if (_use_pullups)
  {
    // Activate internal pullups for TWI.
    pinMode(SDA, INPUT_PULLUP);
    pinMode(SCL, INPUT_PULLUP);
  }
  else
  {
    // Deactivate internal pullups for TWI.
    pinMode(SDA, INPUT);
    pinMode(SCL, INPUT);
  }

  SetFrequency(twi_freq);

  // Enable TWI module and TWI interrupt.
  TWCR = (1 << TWEN) | (_use_isr << TWIE);
  //TWCR = (1 << TWEA) | (1 << TWEN) | (_use_isr << TWIE);  // For Target Receiver mode and Target Transmitter mode.

  wdt_reset();
  WDTCSR = (1 << WDCE) | (1 << WDE);  // Enable WDT changes.
  WDTCSR = (1 << WDP0) | (0 << WDE);  // 32 ms timeout. Stopped. Will be started on a START condition.
}

void TWI::Disable()
{
  cli();

  // Disable WDT.
  wdt_reset();
  WDTCSR = (1 << WDCE) | (1 << WDE);  // Enable WDT changes.
  WDTCSR = (1 << WDP0) | (0 << WDE);  // 32 ms timeout. Stopped. Will be started on a START condition.

  // Disable TWI ACK, TWI module and TWI interrupt.
  TWCR &= ~((1 << TWEA) | (1 << TWEN) | (1 << TWIE));

  sei();

  // Deactivate internal pullups for TWI.
  pinMode(SDA, INPUT);
  pinMode(SCL, INPUT);
}

void TWI::SetOwnTargetAddress(const byte own_target_address, const bool general_call_address)
{
  TWAR = own_target_address << 1 | general_call_address;
}

uint32_t TWI::SetFrequency(uint32_t twi_freq)
{
  if (_smbus_mode)
  {
    // SMBus clock frequency is 10 kHz to 1 MHz for SMBus v3.1.
    // SMBus clock frequency is 10 kHz to 100 kHz for SMBus v1.1.
    /**
    if (twi_freq > 100000u)
    {
    twi_freq = 100000u;
    }
    else
    /**/
    if (twi_freq < 10000u)
    {
      twi_freq = 10000u;
    }
  }
  if (twi_freq > 400000u)
  {
    // Up to 400 kHz data transfer speed.
    // §22.1 Features
    twi_freq = 400000u;
  }

  // TWI prescaler of 1.
  // Table 22-8. TWI Bit Rate Prescaler
  // TWPS1 = 0, TWPS0 = 0
  TWSR &= 0b11111100;

  // Achievable maximum TWI frequency with current CPU frequency, CLK prescaler and minimum TWBR with TWI prescaler of 1.
  // Rounded down to nearest integer.
  const uint32_t TWI_FREQ_MAX = F_CPU / ( ( 1 << CLKPR ) * ( 16 + 2 * 0x00 * 1 ) );

  // Achievable minimum TWI frequency with current CPU frequency, CLK prescaler and maximum TWBR with TWI prescaler of 1.
  // Rounded up to nearest integer.
  const uint32_t DIVISOR = ( ( 1 << CLKPR ) * ( 16 + 2 * 0xFF * 1 ) );
  const uint32_t TWI_FREQ_MIN = (F_CPU + (DIVISOR - 1)) / DIVISOR;

  if (twi_freq > TWI_FREQ_MAX)
  {
    twi_freq = TWI_FREQ_MAX;
  }
  else if (twi_freq < TWI_FREQ_MIN)
  {
    twi_freq = TWI_FREQ_MIN;
  }

  // TWI bit rate.
  // §22.5.2 Bit Rate Generator Unit
  // §9.12.2 CLKPR – Clock Prescale Register
  const byte bit_rate = (F_CPU / (1 << CLKPR) / twi_freq - 16) / 2;

  TWBR = bit_rate;

  // Check.
  const uint32_t twi_freq_check = F_CPU / ( ( 1 << CLKPR ) * ( 16 + 2 * TWBR * 1 ) );

  return twi_freq_check;
}

bool TWI::Transaction(const Sequences sequence, const byte target_address, const byte internal_address, const byte internal_address_size, const byte *buffer, const uint16_t count, const Modes mode = Modes::Continue)
{
  if (internal_address_size > 3 || buffer == nullptr || count < 1)
  {
    return false;
  }
  cli();
  States s = _state;
  sei();
  if (s == States::Busy)
  {
    return false;
  }
  _state = States::Busy;
  _result = Results::Unknown;
  _sequence = sequence;
  _use_isr = mode == Modes::Continue ? true : false;
  _target_address_read = target_address << 1 | 1;
  _target_address_write = target_address << 1;
  _internal_address = internal_address & 0x00ffffff;
  _internal_address_size = internal_address_size > 3 ? 3 : internal_address_size;
  _buffer = buffer;
  _count = count;
  _index = 0;
  _timeout = false;
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (_use_isr << TWIE);  // START.
  wdt_reset();
  WDTCSR = 1 << WDIE;  // Enable WDT interrupt. Enabled on START.

  if (mode == Modes::Continue)
  {
    return true;  // Let the TWI ISR handle the rest of the transaction.
  }

  // Handle the rest of the transaction with a blocking while loop.
  while ((_state == States::Busy) && !_timeout)
  {
    // Wait for TWINT to go high or a timeout to occur before updating the state machine.
    while ((TWCR & (1 << TWINT)) == 0 && !_timeout) continue;

    UpdateStateMachine();  // TWI non-ISR mode.
  }

  return true;
}

//
// Building-block functions that enable the creation of custom sequences.
//

void TWI::SendStart()
{
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  wdt_reset();
  WDTCSR = 1 << WDIE;  // Enable WDT interrupt. Enabled on START.
  while ((TWCR & (1 << TWINT)) == 0 && !_timeout) continue;
}

void TWI::SendTargetAddressWrite(const byte target_address)
{
  TWDR = target_address << 1;  // R/nW bit is 0.
  TWCR = (1 << TWINT) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0 && !_timeout) continue;
  wdt_reset();
}

void TWI::SendTargetAddressRead(const byte target_address)
{
  TWDR = target_address << 1 | 1;  // R/nW bit is 1.
  TWCR = (1 << TWINT) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0 && !_timeout) continue;
  wdt_reset();
}

void TWI::SendData(const byte data)
{
  TWDR = data;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0 && !_timeout) continue;
  wdt_reset();
}

byte TWI::ReceiveDataSendAck()
{
  TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0 && !_timeout) continue;
  wdt_reset();
  return TWDR;
}

byte TWI::ReceiveDataSendNotAck()
{
  TWCR = (1 << TWINT) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0 && !_timeout) continue;
  wdt_reset();
  return TWDR;
}

void TWI::SendStop()
{
  TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
  
  // TWINT is not set after STOP, so poll TWSR for the result (takes 5 us).
  while (static_cast<Status>(TWSR & 0xF8) != Status::xF8_Complete && static_cast<Status>(TWSR & 0xF8) != Status::x00_BusError && !_timeout) continue;

  wdt_reset();

  // Disable WDT interrupt.
  WDTCSR = (1 << WDCE) | (1 << WDE);  // Enable WDT changes.
  WDTCSR = (1 << WDP0) | (0 << WDE);  // 32 ms timeout. Stopped.
}

void TWI::ReleaseBus()
{
  TWCR = (1 << TWINT) | (1 << TWEN);
  //TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);  // For Target Receiver mode and Target Transmitter mode.
}

//
// Interrupt Service Routines for watchdog timer and TWI.
//

void TWI::SendStopFromTwiIsr()
{
  TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN) | (_use_isr << TWIE);

  // TWINT is not set after STOP, so poll TWSR for the result (takes 5 us).
  while (static_cast<Status>(TWSR & 0xF8) != Status::xF8_Complete && static_cast<Status>(TWSR & 0xF8) != Status::x00_BusError && !_timeout) continue;

  wdt_reset();

  // Disable WDT interrupt.
  WDTCSR = (1 << WDCE) | (1 << WDE);  // Enable WDT changes.
  WDTCSR = (1 << WDP0) | (0 << WDE);  // 32 ms timeout. Stopped.

  _state = States::Ready;
}

void TWI::HandleTimeout()
{
  // Save bitrate and address settings.
  uint8_t previous_TWBR = TWBR;
  uint8_t previous_TWAR = TWAR;

  // Disable TWI ACK, TWI module and TWI interrupt.
  TWCR &= ~((1 << TWEA) | (1 << TWEN) | (1 << TWIE));

  if (_use_pullups)
  {
    // Activate internal pullups for TWI.
    pinMode(SDA, INPUT_PULLUP);
    pinMode(SCL, INPUT_PULLUP);
  }
  else
  {
    // Deactivate internal pullups for TWI.
    pinMode(SDA, INPUT);
    pinMode(SCL, INPUT);
  }

  // Restore the previous bitrate and address settings.
  TWAR = previous_TWAR;
  TWBR = previous_TWBR;

  // Enable TWI module and TWI interrupt.
  TWCR = (1 << TWEN) | (_use_isr << TWIE);
  //TWCR = (1 << TWEA) | (1 << TWEN) | (_use_isr << TWIE);  // For Target Receiver mode and Target Transmitter mode.
}

void TWI::UpdateStateMachine()
{
  if (_timeout || _state != States::Busy)
  {
    return;
  }

  // Test Two-Wire Status Register.
  switch (static_cast<Status>(TWSR & 0xF8))
  {
    //
    // Table 22-7. Miscellaneous states
    //

    case Status::x00_BusError:

      // Bus error due to an illegal START or STOP condition.
      // No TWDR action. Only the internal hardware is affected,
      // no STOP condition is sent on the bus.
      // In all cases, the bus is released and TWSTO is cleared.
      //TWCR = 0b10010101;
      //TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN) | (_use_isr << TWIE);
      SendStopFromTwiIsr();

      break;

    case Status::xF8_Complete:

      // No relevant state information available; TWINT = “0”.
      // No TWDR action. No TWCR action.
      // Wait or proceed current transfer.

      wdt_reset();

      // Disable WDT interrupt.
      WDTCSR = (1 << WDCE) | (1 << WDE);  // Enable WDT changes.
      WDTCSR = (1 << WDP0) | (0 << WDE);  // 32 ms timeout. Stopped.

      _state = States::Ready;

      break;

    //
    // Common status codes:
    //  Table 22-3. Status Codes for Controller Transmitter Mode
    //  Table 22-4. Status Codes for Controller Receiver Mode
    //

    case Status::x08_StartTransmitted:

      // SMBus datasheet §4.2.3 Controller device clock extension definitions and conditions
      // Reset timestamp.
      wdt_reset();

      // Load TargetAddress+W. Expect ACK or NOT ACK.

      // Assign Twin-Wire Data Register with target address and write bit.
      TWDR = _target_address_write;

      // Reset TWINT bit of Twin-Wire Control Register by writing 1 to it.
      //TWCR = 0b10000101;
      TWCR = (1 << TWINT) | (1 << TWEN) | (_use_isr << TWIE);

      break;

    case Status::x10_RepeatedStartTransmitted:

      // SMBus datasheet §4.2.3 Controller device clock extension definitions and conditions
      // Reset timestamp.
      wdt_reset();

      // Option 1: Load TargetAddress+W. Expect ACK or NOT ACK.
      //TWDR = _target_address_write;
      //TWCR = 0b10000101;
      //TWCR = (1 << TWINT) | (1 << TWEN) | (_use_isr << TWIE);

      // Option 2: Load TargetAddress+R. Expect ACK or NOT ACK.
      TWDR = _target_address_read;
      //TWCR = 0b10000101;
      TWCR = (1 << TWINT) | (1 << TWEN) | (_use_isr << TWIE);

      break;

    case Status::x38_ArbitrationLost:
      // For Controller Transmitter mode, arbitration lost in TargetAddress+W or data bytes.
      // For Controller Receiver mode, arbitration lost in TargetAddress+R or NOT ACK bit.

      // Option 1: No TWDR action. 2-wire Serial Bus will be released and not addressed Target mode entered.
      //TWCR = 0b10000101;
      TWCR = (1 << TWINT) | (1 << TWEN) | (_use_isr << TWIE);

      // Option 2: No TWDR action. A START condition will be transmitted when the bus becomes free.
      //TWCR = 0b10100101;
      //TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (_use_isr << TWIE);    // Try again.

      break;

    //
    // Table 22-3. Status Codes for Controller Transmitter Mode
    //

    case Status::x18_TargetAddressWriteTransmittedAckReceived:

      // SMBus datasheet §4.2.3 Controller device clock extension definitions and conditions
      // Reset timestamp.
      wdt_reset();

      // Option 1: Load data byte. Expect ACK or NOT ACK.
      if (_internal_address_size > 0)
      {
        _internal_address_size--;
        TWDR = _internal_address >> (_internal_address_size * 8);
      }
      else
      {
        _index = 0;
        TWDR = _buffer[_index++];
      }
      //TWCR = 0b10000101;
      TWCR = (1 << TWINT) | (1 << TWEN) | (_use_isr << TWIE);

      // Option 2: No TWDR action. Repeated START will be transmitted.
      //TWCR = 0b10100101;
      //TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (_use_isr << TWIE);

      // Option 3: No TWDR action. STOP condition will be transmitted. TWSTO Flag will be reset.
      //TWCR = 0b10010101;
      //TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN) | (_use_isr << TWIE);
      //SendStopFromTwiIsr();

      // Option 4: No TWDR action. STOP condition followed by a START condition will be transmitted and TWSTO flag will be reset.
      //TWCR = 0b10110101;
      //TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWSTO) | (1 << TWEN) | (_use_isr << TWIE);

      break;

    case Status::x20_TargetAddressWriteTransmittedNotAckReceived:

      // Option 1: Load data byte. Expect data byte ACK or NOT ACK.
      //TWDR = _data_byte;
      //TWCR = 0b10000101;
      //TWCR = (1 << TWINT) | (1 << TWEN) | (_use_isr << TWIE);

      // Option 2: No TWDR action. Repeated START will be transmitted.
      //TWCR = 0b10100101;
      //TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (_use_isr << TWIE);

      // Option 3: No TWDR action. STOP condition will be transmitted. TWSTO Flag will be reset.
      //TWCR = 0b10010101;
      //TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN) | (_use_isr << TWIE);
      SendStopFromTwiIsr();

      // Option 4: No TWDR action. STOP condition followed by a START condition will be transmitted and TWSTO flag will be reset.
      //TWCR = 0b10110101;
      //TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWSTO) | (1 << TWEN) | (_use_isr << TWIE);   // Try again.

      break;

    case Status::x28_DataByteTransmittedAckReceived:

      // SMBus datasheet §4.2.3 Controller device clock extension definitions and conditions
      // Reset timestamp.
      wdt_reset();

      if (_internal_address_size > 0)
      {
        _internal_address_size--;
        TWDR = _internal_address >> (_internal_address_size * 8);
      }
      else if (_sequence == Sequences::BurstWrite)
      {
        if (_index < _count)
        {
          // Option 1: Load data byte. Expect ACK or NOT ACK.
          TWDR = _buffer[_index++];
          //TWCR = 0b10000101;
          TWCR = (1 << TWINT) | (1 << TWEN) | (_use_isr << TWIE);
        }
        else
        {
          // Option 3: No TWDR action. STOP condition will be transmitted. TWSTO Flag will be reset.
          //TWCR = 0b10010101;
          //TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN) | (_use_isr << TWIE);
          SendStopFromTwiIsr();

          _result = Results::Success;
        }
      }
      else if (_sequence == Sequences::BurstRead)
      {
        // Option 2: No TWDR action. Repeated START will be transmitted.
        //TWCR = 0b10100101;
        TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (_use_isr << TWIE);
      }

      // Option 4: No TWDR action. STOP condition followed by a START condition will be transmitted and TWSTO flag will be reset.
      //TWCR = 0b10110101;
      //TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWSTO) | (1 << TWEN) | (_use_isr << TWIE);

      break;

    case Status::x30_DataByteTransmittedNotAckReceived:

      // Option 1: Load data byte. Expect data byte ACK or NOT ACK.
      //TWDR = _data_byte;
      //TWCR = 0b10000101;
      //TWCR = (1 << TWINT) | (1 << TWEN) | (_use_isr << TWIE);

      // Option 2: No TWDR action. Repeated START will be transmitted.
      //TWCR = 0b10100101;
      //TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (_use_isr << TWIE);

      // Option 3: No TWDR action. STOP condition will be transmitted. TWSTO Flag will be reset.
      //TWCR = 0b10010101;
      //TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN) | (_use_isr << TWIE);
      SendStopFromTwiIsr();

      // Option 4: No TWDR action. STOP condition followed by a START condition will be transmitted and TWSTO flag will be reset.
      //TWCR = 0b10110101;
      //TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWSTO) | (1 << TWEN) | (_use_isr << TWIE);   // Try again.

      break;

    //
    // Table 22-4. Status Codes for Controller Receiver Mode
    //

    case Status::x40_TargetAddressReadTransmittedAckReceived:

      // SMBus datasheet §4.2.3 Controller device clock extension definitions and conditions
      // Reset timestamp.
      wdt_reset();

      _index = 0;

      if (_index < _count - 1)
      {
        // Option 2: No TWDR action. Data byte will be received and ACK will be returned.
        //TWCR = 0b11000101;  // Set EA bit to send ACK.
        TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (_use_isr << TWIE);
      }
      else
      {
        // Option 1: No TWDR action. Data byte will be received and NOT ACK will be returned.
        //TWCR = 0b10000101;  // Clear EA bit to send NOT ACK.
        TWCR = (1 << TWINT) | (1 << TWEN) | (_use_isr << TWIE);
      }

      break;

    case Status::x48_TargetAddressReadTransmittedNotAckReceived:

      // Option 1: No TWDR action. Repeated START will be transmitted.
      //TWCR = 0b10100101;
      //TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (_use_isr << TWIE);

      // Option 2: No TWDR action. STOP condition will be transmitted and TWSTO flag will be reset.
      //TWCR = 0b10010101;
      //TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN) | (_use_isr << TWIE);
      SendStopFromTwiIsr();

      // Option 3: No TWDR action. STOP condition followed by a START condition will be transmitted and TWSTO flag will be reset.
      //TWCR = 0b10110101;
      //TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWSTO) | (1 << TWEN) | (_use_isr << TWIE);   // Try again.

      break;

    case Status::x50_DateByteReceivedAckReturned:

      // SMBus datasheet §4.2.3 Controller device clock extension definitions and conditions
      // Reset timestamp.
      wdt_reset();

      _buffer[_index++] = TWDR;

      if (_index < _count - 1)
      {
        // Option 2: No TWDR action. Data byte will be received and ACK will be returned.
        //TWCR = 0b11000101;  // Set EA bit to send ACK.
        TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (_use_isr << TWIE);
      }
      else
      {
        // Option 1: No TWDR action. Data byte will be received and NOT ACK will be returned.
        //TWCR = 0b10000101;
        TWCR = (1 << TWINT) | (1 << TWEN) | (_use_isr << TWIE);
      }

      break;

    case Status::x58_DataByteReceivedNotAckReturned:

      _buffer[_index] = TWDR;

      // Option 1: No TWDR action. Repeated START will be transmitted.
      //TWCR = 0b10100101;
      //TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (_use_isr << TWIE);

      // Option 2: No TWDR action. STOP condition will be transmitted and TWSTO flag will be reset.
      //TWCR = 0b10010101;
      //TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN) | (_use_isr << TWIE);
      SendStopFromTwiIsr();

      _result = Results::Success;

      // Option 3: No TWDR action. STOP condition followed by a START condition will be transmitted and TWSTO flag will be reset.
      //TWCR = 0b10110101;
      //TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWSTO) | (1 << TWEN) | (_use_isr << TWIE);

      break;

    default:
      break;
  }
}

inline void PFHandleTimeout()
{
  // Disable any further WDT interrrupts.
  // It will be re-enabled at the next START condition.
  WDTCSR = 0;

  TWI::HandleTimeout();

  TWI::_timeout = true;
  TWI::_state = TWI::States::Ready;
  TWI::_result = TWI::Results::Timeout;
}

inline void PFUpdateStateMachine()
{
  TWI::UpdateStateMachine();  // TWI ISR mode.
}

ISR(WDT_vect)
{
  PFHandleTimeout();
}

ISR(TWI_vect)
{
  PFUpdateStateMachine();
}
