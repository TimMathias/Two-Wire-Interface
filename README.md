# Two-Wire Interface
Two-Wire Interface for Arduino – Asynchronous and Synchronous Transactions – Controller and Target modes.

- Asynchronous (non-blocking) transactions using the Two-Wire Interface (TWI) interrupt service routine.
- Synchronous (blocking) transactions by polling the TWI interrupt flag.
- Supports target devices with an internal address register of 1, 2, or 3 bytes.
- 32 ms timeout provided by the Watchdog Timer.
- Buffers are supplied by the caller.
- Transaction limit of 65535 bytes.
- Internal pullup resistors on SCL and SDA are disabled by default.

## Asynchronous transactions (default)

These functions return an `enum` to indicate the status of the transaction. If it fails to start, it returns `TWI::Results::FailedToStart`. If it started, it returns `TWI::Results::Started`.

To find out whether the transaction completed successfully, call `twi.GetResult()` or `twi.IsSuccess()` in `loop()`.

### Read buffer

```c++
twi.Read(target_address, rx_buffer, rx_buffer_size);
```

### Read buffer from internal address

```c++
twi.Read(target_address, internal_address, internal_address_size, rx_buffer, rx_buffer_size)
```

### Write buffer

```c++
twi.Write(target_address, tx_buffer, tx_buffer_size);
```

### Write buffer to internal address

```c++
twi.Write(target_address, internal_address, internal_address_size, tx_buffer, tx_buffer_size);
```

### Write byte

```c++
twi.Write(target_address, data_byte);
```

### Write byte to internal address

```c++
twi.Write(target_address, internal_address, internal_address_size, data_byte);
```

## Synchronous transactions (optional)

Specify the wait flag `TWI::Modes::Wait` to enable synchronous mode, e.g. to read a buffer synchronously:

```c++
twi.Read(target_address, rx_buffer, rx_buffer_size, TWI::Modes::Wait);
```

These functions return an `enum` to indicate the status of the transaction. If it fails to start, it returns `TWI::Results::FailedToStart`. If it completes successfully, it returns `TWI::Results::Success`.

## Internal address register

Some target devices have addressable internal registers for configuration and control information. The internal address size can be 1, 2 or 3 bytes:

| Size of internal address register (bytes) | Internal address range |
| :---: | :---: |
| 1 | 0 to 255 |
| 2 | 0 to 65,535 |
| 3 | 0 to 16,777,215 |

## Timeouts

There is a 32&nbsp;ms timeout provided by the Watchdog Timer to detect when the I<sup>2</sup>C bus freezes. When triggered, the TWI will be reset.

## Buffers

The caller supplies the buffer and the buffer size. This provides greater flexibility, e.g. a display can be double buffered using asynchronous transactions where the second buffer can be populated with data in `loop()` while the first buffer is being sent to the display by the TWI ISR.

## Internal pullup resistors

The internal pullup resistors on SCL and SDA are disabled by default. To enable them, call `twi.Enable(twi_frequency, pullups)` with the second parameter set to true.

## Example

This simulator example demonstrates how to communicate with a Motion Processor Unit &ndash; the MPU-6050 &ndash; on the I<sup>2</sup>C bus. It also demonstrates recovery from a communication error, i.e. press the buttons to freeze the I<sup>2</sup>C bus, whence upon releasing them, the TWI resets and continues with normal communication: [TWI MPU-6050 Test](https://wokwi.com/projects/306825661000974912):

 [![image](https://user-images.githubusercontent.com/67882955/166562996-37a462e0-a560-43ea-b4a0-ab68c4e13ea0.png)](https://wokwi.com/projects/306825661000974912)




