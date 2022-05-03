#pragma once
#ifndef TWI_H
#define TWI_H

//
// Asynchronous Two-Wire Interface (IIC or I2C) for ATmega48A/PA/88A/PA/168A/PA/328/P
//
// Implements non-blocking (aynchronous) burst-write and burst-read using the TWI ISR and WDT ISR.
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

#include <Arduino.h>
#include <avr/wdt.h>

class TWI
{
  private:

    enum class Status : byte
    {
      //
      // Table 22-7. Miscelleaneous States
      //
      x00_BusError = 0x00,
      xF8_Complete = 0xF8,

      //
      // Common status codes:
      //  Table 22-3. Status Codes for Controller Transmitter Mode
      //  Table 22-4. Status Codes for Controller Receiver Mode
      //
      x08_StartTransmitted = 0x08,
      x10_RepeatedStartTransmitted = 0x10,
      x38_ArbitrationLost = 0x38,

      //
      // Table 22-3. Status Codes for Controller Transmitter Mode
      //
      x18_TargetAddressWriteTransmittedAckReceived = 0x18,
      x20_TargetAddressWriteTransmittedNotAckReceived = 0x20,
      x28_DataByteTransmittedAckReceived = 0x28,
      x30_DataByteTransmittedNotAckReceived = 0x30,

      //
      // Table 22-4. Status Codes for Controller Receiver Mode
      //
      x40_TargetAddressReadTransmittedAckReceived = 0x40,
      x48_TargetAddressReadTransmittedNotAckReceived = 0x48,
      x50_DateByteReceivedAckReturned = 0x50,
      x58_DataByteReceivedNotAckReturned = 0x58
    };

  public:

    enum class Modes : bool
    {
      Continue,  // Returns to the caller after starting the transaction. The TWI ISR continues the transaction to completion asynchronously.
      Wait       // Conducts the transaction synchronously without the TWI ISR by polling the TWINT flag.
    };

  private:

    enum class Sequences : byte
    {
      BurstWrite,  // Tx Start, Tx TargetAddress+W, Rx Ack, [Optional: Tx internal address, Rx Ack,] (Tx data and Rx Ack) x N, Tx Stop.
      BurstRead    // Tx Start, Tx TargetAddress+W, Rx Ack, [Optional: Tx internal address, Rx Ack,] Tx Repeated start, Tx TargetAddress+R, Rx Ack, (Rx data and Tx Ack) x (N - 1), Rx data N, Tx Not Ack, Tx Stop.
    };

  public:

    // State machine feedback to the caller.
    enum class States : byte
    {
      Ready,
      Busy
    };

    // Result of sequence.
    enum class Results : byte
    {
      Unknown,
      Timeout,
      Success
    };

  private:

    static Sequences _sequence;

    static volatile States _state;
    static volatile Results _result;

    // Internal pullup resistors on SCL and SDA
    //   true : enable them.
    //   false: disable them.
    static volatile bool _use_pullups;

    // TWI ISR
    //   true : Asynchronous (non-blocking) transactions using the TWI ISR.
    //   false: Synchronous (blocking) transactions by polling the TWINT flag.
    static volatile bool _use_isr;

    static volatile byte *_buffer;               // Pointer to caller's buffer.
    static volatile uint16_t _count;             // Number of bytes to transfer.
    static volatile uint16_t _index;             // Index into caller's buffer.
    static volatile byte _target_address_read;   // Composite 7-bit target address merged with read bit (1).
    static volatile byte _target_address_write;  // Composite 7-bit target address merged with write bit (0).
    static volatile uint32_t _internal_address;  // Internal register to read/write.
    static volatile byte _internal_address_size; // Size of internal register address: 1 to 3 bytes. 0 means there is no internal address.
    static volatile byte _data_byte;             // data_byte to read/write.

    // Timeout to avoid infinite loop if the TWI bus freezes.
    //
    // Timeout detection is provided by the watchdog timer.
    // The WDT operates in interrupt mode with a timeout of 32 ms.
    // It is enabled on a START condition and disabled at the end of the transaction or when it triggers (whichever is sooner).
    // The transaction sequence resets the WDT at appropriate times to prevent it triggering.
    // If the TWI bus freezes, the WDT will trigger after 32 ms of inactivity.
    //
    // SMBus datasheet §4.2.3 Controller device clock extension definitions and conditions
    // SMBus datasheet Table 2. SMBus AC specifications
    // tTIMEOUT  Detect clock low timeout:  min 25 ms;  max 35 ms.  See Note 3.
    // Note 3: Devices participating in a transfer can abort the transfer in progress and release the bus when any single clock low interval exceeds the value of tTIMEOUT,MIN. After the controller in a transaction detects this condition, it must generate a stop condition within or after the current data byte in the transfer process. Devices that have detected this condition must reset their communication and be able to receive a new START condition no later than tTIMEOUT,MAX. Typical device examples include the host controller, and embedded controller, and most devices that can controller the SMBus. Some simple devices do not contain a clock low drive circuit; this simple kind of device typically may reset its communications port after a start or a stop condition.
    //
    // tLOW:SEXT  Cumulative clock low extend time (target device): 25 ms.  See Note 5.
    // Note 5: tLOW:SEXT is the cumulative time a given target device is allowed to extend the clock cycles in one message from the initial START to the STOP. It is possible that another target device or the controller will also extend the clock causing the combined clock low extend time to be greater than tLOW:SEXT. Therefore, this parameter is measured with the target device as the sole target of a full-speed controller.
    //
    // tLOW:MEXT  Cumulative clock low extend time (controller device): 10 ms.  See Note 6.
    // Note 6: tLOW:MEXT is the cumulative time a controller device is allowed to extend its clock cycles within each byte of a message as defined from START-to-ACK, ACK-to-ACK, or ACK-to-STOP. It is possible that a target device or another controller will also extend the clock causing the combined clock low time to be greater than tLOW:MEXT on a given byte. This parameter is measured with a full speed target device as the sole target of the controller.
    //static volatile uint16_t mext_timestamp;  // Timestamp at beginning of transaction.
    //static uint16_t mext_timeout = 10000u;    // Timeout value in microseconds. Tlow:mext for SMBus.
    static volatile bool _timeout;              // Flag to indicate timeout condition.

    static bool _smbus_mode;

  public:

    TWI()
    {
    }

    void Enable(const uint32_t twi_freq = 400000u, const bool use_pullups = false);//, const uint16_t twi_mext_timeout = 10000u);

    void Disable();

    void SetOwnTargetAddress(const byte own_target_address, const bool general_call_address);

    uint32_t SetFrequency(uint32_t twi_freq);

    States GetState()
    {
      // Get the progress of the state machine.
      cli();
      States s = _state;
      sei();
      return s;
    }

    Results GetResult()
    {
      // Get the result of the state machine.
      cli();
      Results r = _result;
      sei();
      return r;
    }

    bool IsComplete()
    {
      cli();
      States s = _state;
      Results r = _result;
      sei();
      bool complete = s == States::Ready && (r == Results::Success || r == Results::Timeout);
      return complete;
    }

    bool IsReady()
    {
      cli();
      States s = _state;
      sei();
      return s == States::Ready;
    }

    bool IsSuccess()
    {
      cli();
      States s = _state;
      Results r = _result;
      sei();
      return s == States::Ready && r == Results::Success;
    }

  public:

    //
    // Transaction functions for burst write and burst read:
    //   Modes::Continue  Uses the non-blocking TWI ISR (default).
    //   Modes::Wait      Uses a blocking while loop.
    //

    // BurstWrite: Tx Start, Tx TargetAddress+W, Rx Ack, (Tx data and Rx Ack) x N, Tx Stop.
    bool Write(const byte target_address, const byte *buffer, const uint16_t count, const Modes mode = Modes::Continue)
    {
      return Transaction(Sequences::BurstWrite, target_address, 0, 0, buffer, count, mode);
    }

    // BurstWrite: Tx Start, Tx TargetAddress+W, Rx Ack, Tx internal address, Rx Ack, (Tx data and Rx Ack) x N, Tx Stop.
    bool Write(const byte target_address, const uint32_t internal_address, const byte internal_address_size, const byte *buffer, const uint16_t count, const Modes mode = Modes::Continue)
    {
      return Transaction(Sequences::BurstWrite, target_address, internal_address, internal_address_size, buffer, count, mode);
    }

    // BurstWrite: Tx Start, Tx TargetAddress+W, Rx Ack, (Tx data and Rx Ack) x N, Tx Stop.
    bool Write(const byte target_address, const byte data, const Modes mode = Modes::Continue)
    {
      cli();
      States s = _state;
      sei();
      if (s == States::Busy)
      {
        return false;
      }
      _data_byte = data;
      return Transaction(Sequences::BurstWrite, target_address, 0, 0, &_data_byte, 1, mode);
    }

    // BurstWrite: Tx Start, Tx TargetAddress+W, Rx Ack, Tx internal address, Rx Ack, (Tx data and Rx Ack) x N, Tx Stop.
    bool Write(const byte target_address, const uint32_t internal_address, const byte internal_address_size, const byte data, const Modes mode = Modes::Continue)
    {
      cli();
      States s = _state;
      sei();
      if (s == States::Busy)
      {
        return false;
      }
      _data_byte = data;
      return Transaction(Sequences::BurstWrite, target_address, internal_address, internal_address_size, &_data_byte, 1, mode);
    }

    // BurstRead: Tx Start, Tx TargetAddress+W, Rx Ack, Tx Repeated start, Tx TargetAddress+R, Rx Ack, (Rx data and Tx Ack) x (N - 1), Rx data N, Tx Not Ack, Tx Stop.
    bool Read(const byte target_address, const byte *buffer, const uint16_t count, const Modes mode = Modes::Continue)
    {
      return Transaction(Sequences::BurstRead, target_address, 0, 0, buffer, count, mode);
    }

    // BurstRead: Tx Start, Tx TargetAddress+W, Rx Ack, Tx internal address, Rx Ack, Tx Repeated start, Tx TargetAddress+R, Rx Ack, (Rx data and Tx Ack) x (N - 1), Rx data N, Tx Not Ack, Tx Stop.
    bool Read(const byte target_address, const uint32_t internal_address, const byte internal_address_size, const byte *buffer, const uint16_t count, const Modes mode = Modes::Continue)
    {
      return Transaction(Sequences::BurstRead, target_address, internal_address, internal_address_size, buffer, count, mode);
    }

  private:

    bool Transaction(const Sequences sequence, const byte target_address, const byte internal_address, const byte internal_address_size, const byte *buffer, const uint16_t count, const Modes mode = Modes::Continue);

  public:

    //
    // Building-block functions that enable the creation of custom sequences.
    //

    void SendStart();
    void SendTargetAddressWrite(const byte target_address);
    void SendTargetAddressRead(const byte target_address);
    void SendData(const byte data);
    byte ReceiveDataSendAck();
    byte ReceiveDataSendNotAck();
    void SendStop();
    void ReleaseBus();

  private:

    static void HandleTimeout()
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

    static void UpdateStateMachine()
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
          TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN) | (_use_isr << TWIE);

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
          TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN) | (_use_isr << TWIE);

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
              TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN) | (_use_isr << TWIE);

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
          TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN) | (_use_isr << TWIE);

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
          TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN) | (_use_isr << TWIE);

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
          TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN) | (_use_isr << TWIE);

          _result = Results::Success;

          // Option 3: No TWDR action. STOP condition followed by a START condition will be transmitted and TWSTO flag will be reset.
          //TWCR = 0b10110101;
          //TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWSTO) | (1 << TWEN) | (_use_isr << TWIE);

          break;

        default:
          break;
      }
    }

    friend void PFHandleTimeout();       // Called by WDT ISR.
    friend void PFUpdateStateMachine();  // Called by TWI ISR.
};

extern TWI twi;

#endif
