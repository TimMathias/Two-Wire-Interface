#pragma once
#ifndef TWI_H
#define TWI_H

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
// Copyright (c) 2021-2023 Timothy Mathias
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

#include "queue.h"

#define BYTE_BUFFER_LENGTH 64

#define nDEBUG_TWI

#ifdef DEBUG_TWI
  #define DebugPrint(...) Serial.print(__VA_ARGS__)
  #define DebugPrintln(...) Serial.println(__VA_ARGS__)
#else
  #define DebugPrint(...)
  #define DebugPrintln(...)
#endif

// Two-Wire Interface.
class TWI
{
  public:

    // Transaction modes.
    enum class Modes : bool
    {
      Continue,  // Returns to the caller after starting the transaction. The TWI ISR continues the transaction to completion asynchronously.
      Wait       // Waits for the TWI ISR to complete the transaction.
    };

    // Transaction sequences.
    enum class Sequences : byte
    {
      BurstRead,        // Tx Start, Tx TargetAddress+R, Rx Ack, (Rx data and Tx Ack) x (N - 1), Rx data N, Tx Not Ack, Tx Stop.
                        // Tx Start, Tx TargetAddress+W, Rx Ack, Tx internal address, Rx Ack, Tx Repeated start, Tx TargetAddress+R, Rx Ack, (Rx data and Tx Ack) x (N - 1), Rx data N, Tx Not Ack, Tx Stop.
      BurstWrite,       // Tx Start, Tx TargetAddress+W, Rx Ack, [Optional: Tx internal address, Rx Ack,] (Tx data and Rx Ack) x N, Tx Stop.
    };

    // Result of transaction.
    enum class Results : byte
    {
      Unknown           = 10, // Unkown before attempting to start a transaction.
      Pending           = 20, // Transaction waiting to be started.
      FailedToStart     = 30, // Transaction failed to start.
      Started           = 41, // Transaction started.
      ArbitrationLost   = 52, // Arbitration lost during transaction.
      Timeout           = 62, // Timeout during transaction.
      Success           = 72  // Transaction completed successfully.
    };

  private:

    // TWSR values.
    enum class Status : byte
    {
      //
      // Table 22-6. Miscelleaneous States
      //

      x00_BusError = 0x00,
      xF8_Complete = 0xF8,

      //
      // Common status codes:
      //   Table 22-2. Status Codes for Controller Transmitter Mode
      //   Table 22-3. Status Codes for Controller Receiver Mode
      //

      x08_StartTransmitted = 0x08,
      x10_RepeatedStartTransmitted = 0x10,
      x38_ArbitrationLost = 0x38,

      //
      // Table 22-2. Status Codes for Controller Transmitter Mode
      //

      x18_TargetAddressWriteTransmitted_AckReceived = 0x18,
      x20_TargetAddressWriteTransmitted_NotAckReceived = 0x20,
      x28_DataByteTransmitted_AckReceived = 0x28,
      x30_DataByteTransmitted_NotAckReceived = 0x30,

      //
      // Table 22-3. Status Codes for Controller Receiver Mode
      //

      x40_TargetAddressReadTransmitted_AckReceived = 0x40,
      x48_TargetAddressReadTransmitted_NotAckReceived = 0x48,
      x50_DateByteReceived_AckReturned = 0x50,
      x58_DataByteReceived_NotAckReturned = 0x58,

      //
      // Table 22-4. Status Codes for Target Receiver Mode
      //

      x60_OwnTargetAddressWriteReceived_AckReturned = 0x60,
      x68_ArbitrationLostInTargetAddressAsController_OwnTargetAddressWriteReceived_AckReturned = 0x68,
      x70_GeneralCallAddressReceived_AckReturned = 0x70,
      x78_ArbitrationLostInTargetAddressAsController_GeneralCallAddressReceived_AckReturned = 0x78,
      x80_OwnTargetAddressWrite_DataByteReceived_AckReturned = 0x80,
      x88_OwnTargetAddressWrite_DataByteReceived_NotAckReturned = 0x88,
      x90_GeneralCallAddress_DataByteReceived_AckReturned = 0x90,
      x98_GeneralCallAddress_DataByteReceived_NotAckReturned = 0x98,
      xA0_StopOrRepeatedStartReceivedAsTarget = 0xA0,

      //
      // Table 22-5. Status Codes for Target Transmitter Mode
      //

      xA8_OwnTargetAddressReadReceived_AckReturned = 0xA8,
      xB0_ArbitrationLostInTargetAddressAsController_OwnTargetAddressReadReceived_AckReturned = 0xB0,
      xB8_DataByteTransmitted_AckReceived = 0xB8,
      xC0_DataByteTransmitted_NotAckReceived = 0xC0,
      xC8_LastDataByteTransmitted_AckReceived = 0xC8
    };

  public:

    // State machine feedback to the caller.
    enum class States : byte
    {
      Ready,  // State machine is ready to conduct a transaction.
      Busy    // State machine is busy conducting a transaction.
    };

  private:

    static Sequences _sequence;

    static volatile States _state;
    static volatile Results _result;

    // Internal pullup resistors on SCL and SDA
    //   true : enable them.
    //   false: disable them.
    static volatile bool _use_pullups;

    //
    // Controller Receiver and Controller Transmitter.
    //

    static volatile byte *_controller_buffer;       // Pointer to caller's buffer.
    static volatile uint16_t _controller_count;     // Number of bytes to transfer.
    static volatile uint16_t _controller_index;     // Index into caller's buffer.
    static volatile byte _target_address_read;      // Composite 7-bit target address merged with read bit (1).
    static volatile byte _target_address_write;     // Composite 7-bit target address merged with write bit (0).
    static volatile uint32_t _internal_address;     // Internal register to read/write.
    static volatile byte _internal_address_size;    // Size of internal register address: 1 to 3 bytes. 0 means there is no internal address.
    static byte _data_byte;                         // Data byte to read/write.

    //
    // Own Target Receiver.
    //

    static Queue<byte, BYTE_BUFFER_LENGTH> *_own_target_rx_buffer;

    //
    // Own Target Transmitter.
    //

    static Queue<byte, BYTE_BUFFER_LENGTH> *_own_target_tx_buffer;


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

    // 0 = Clock frequency does NOT conform to SMBus.
    // 1 = Clock frequency conforms to SMBus.
    static bool _smbus_mode;

  public:

    TWI()
    {
    }

    // Enable TWI.
    void Enable(const uint32_t twi_freq = 400000u, const bool use_pullups = false);//, const uint16_t twi_mext_timeout = 10000u);

    // Disable TWI.
    void Disable();

    // Set TWI clock bus frequency.
    uint32_t SetFrequency(uint32_t twi_freq);

    // Initialise own target receiver and transmitter.
    void OwnTargetInit(const byte own_target_address, const bool general_call_address, Queue<byte, BYTE_BUFFER_LENGTH> *rx_buffer, Queue<byte, BYTE_BUFFER_LENGTH> *tx_buffer);

    // Read a byte from own target receiver buffer.
    //   Returns false if nothing to read.
    //   Returns true if it read a byte.
    bool OwnTargetRead(byte &b);

    // Return number of bytes in own target receiver buffer.
    byte OwnTargetReadable() const;

    // Write a byte to own target transmitter buffer.
    bool OwnTargetWrite(byte b);

    // Returns free space in own target transmitter buffer.
    byte OwnTargetWriteable() const;

    // Get the progress of the transaction.
    States GetState()
    {
      return _state;
    }

    // Get the result of the transaction.
    Results GetResult()
    {
      return _result;
    }

    // Has the transaction stopped?
    // i.e. successfully started,
    // but then stopped either successfully or due to a problem.
    bool HasStopped()
    {
      return static_cast<byte>(_result) & 2;
    }

    // Is the state machine ready?
    bool IsReady()
    {
      return _state == States::Ready;
    }

    // Did the transaction complete successfully?
    bool IsSuccess()
    {
      return _result == Results::Success;
    }

  public:

    //
    // Transaction functions for burst read and burst write:
    //   Modes::Continue  Uses the non-blocking TWI ISR (default).
    //   Modes::Wait      Uses a blocking while loop.
    //

    // BurstRead: Tx Start, Tx TargetAddress+R, Rx Ack, (Rx data and Tx Ack) x (N - 1), Rx data N, Tx Not Ack, Tx Stop.
    Results Read(const byte target_address, volatile byte *buffer, const uint16_t count, const Modes mode = Modes::Continue)
    {
      return Transaction(Sequences::BurstRead, target_address, 0, 0, buffer, count, mode);
    }

    // BurstRead: Tx Start, Tx TargetAddress+W, Rx Ack, Tx internal address, Rx Ack, Tx Repeated start, Tx TargetAddress+R, Rx Ack, (Rx data and Tx Ack) x (N - 1), Rx data N, Tx Not Ack, Tx Stop.
    Results Read(const byte target_address, const uint32_t internal_address, const byte internal_address_size, volatile byte *buffer, const uint16_t count, const Modes mode = Modes::Continue)
    {
      return Transaction(Sequences::BurstRead, target_address, internal_address, internal_address_size, buffer, count, mode);
    }

    // BurstWrite: Tx Start, Tx TargetAddress+W, Rx Ack, (Tx data and Rx Ack) x N, Tx Stop.
    Results Write(const byte target_address, volatile byte *buffer, const uint16_t count, const Modes mode = Modes::Continue)
    {
      return Transaction(Sequences::BurstWrite, target_address, 0, 0, buffer, count, mode);
    }

    // BurstWrite: Tx Start, Tx TargetAddress+W, Rx Ack, Tx internal address, Rx Ack, (Tx data and Rx Ack) x N, Tx Stop.
    Results Write(const byte target_address, const uint32_t internal_address, const byte internal_address_size, volatile byte *buffer, const uint16_t count, const Modes mode = Modes::Continue)
    {
      return Transaction(Sequences::BurstWrite, target_address, internal_address, internal_address_size, buffer, count, mode);
    }

    // BurstWrite: Tx Start, Tx TargetAddress+W, Rx Ack, (Tx data and Rx Ack) x N, Tx Stop.
    Results Write(const byte target_address, const byte data, const Modes mode = Modes::Continue)
    {
      States s = _state;
      if (s == States::Busy)
      {
        return Results::FailedToStart;
      }
      _data_byte = data;
      return Transaction(Sequences::BurstWrite, target_address, 0, 0, &_data_byte, 1, mode);
    }

    // BurstWrite: Tx Start, Tx TargetAddress+W, Rx Ack, Tx internal address, Rx Ack, (Tx data and Rx Ack) x N, Tx Stop.
    Results Write(const byte target_address, const uint32_t internal_address, const byte internal_address_size, const byte data, const Modes mode = Modes::Continue)
    {
      States s = _state;
      if (s == States::Busy)
      {
        return Results::FailedToStart;
      }
      _data_byte = data;
      return Transaction(Sequences::BurstWrite, target_address, internal_address, internal_address_size, &_data_byte, 1, mode);
    }

  private:

    Results Transaction(const Sequences sequence, const byte target_address, const uint32_t internal_address, const byte internal_address_size, volatile byte *buffer, const uint16_t count, const Modes mode = Modes::Continue);

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

    //
    // Interrupt Service Routines for watchdog timer and TWI.
    //

    static inline void SendStopFromTwiIsr();
    static inline void HandleTimeout();
    static inline void UpdateStateMachine();
    friend void PFHandleTimeout();       // Called by WDT ISR.
    friend void PFUpdateStateMachine();  // Called by TWI ISR.
};

extern TWI twi;

#endif
