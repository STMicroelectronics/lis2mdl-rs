# LIS2MDL Magnetometer Data Acquisition on STM32F401RE Nucleo-64

This example demonstrates how to interface the **LIS2MDL** magnetometer sensor with an **STM32F401RE** microcontroller board using I2C communication. The program reads magnetic field and temperature data from the sensor in polling mode and outputs the results over UART.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** LIS2MDL Magnetometer
- **Communication Interface:** I2C1 at 100 kHz Standard Mode
- **UART:** USART2 for serial output at 115200 baud

### Default Pin Configuration

| Signal    | STM32F401RE Pin | Description                    |
|-----------|-----------------|--------------------------------|
| I2C1_SCL  | PB8             | I2C clock line (open-drain)   |
| I2C1_SDA  | PB9             | I2C data line (open-drain)    |
| USART2_TX | PA2             | UART transmit for debug output|

The LIS2MDL sensor is connected to the STM32F401RE via I2C1 on pins PB8 (SCL) and PB9 (SDA). UART output is routed through PA2 for serial communication.

---

## Code Description

### Initialization

- The program initializes microcontroller peripherals including clocks, GPIO pins, I2C, and UART.
- I2C1 is configured for 100 kHz Standard Mode with open-drain pins PB8 and PB9.
- UART is configured on PA2 at 115200 baud for serial output.
- A delay abstraction is created using the system timer.

### Sensor Configuration

- The LIS2MDL sensor is initialized over I2C with the default I2C address.
- The device ID is read and printed over UART to verify sensor presence.
- The sensor is reset to default configuration and the program waits until reset completes.
- Block Data Update is enabled to ensure consistent sensor data reads.
- Output data rate is set to 10 Hz.
- Set/Reset sensor mode is configured to cancel offset every output data rate cycle.
- Temperature compensation is enabled.
- The sensor is set to continuous operating mode.

### Data Acquisition Loop

- The program continuously polls the sensor for new magnetic data availability.
- When new data is ready, raw magnetic field data is read and converted to milliGauss (mG).
- Temperature data is also read and converted to degrees Celsius.
- Both magnetic field and temperature values are printed over UART every second.

---

## Usage

1. Connect the LIS2MDL sensor to the STM32F401RE Nucleo board via I2C1 (PB8/PB9).
2. Build and flash the firmware onto the STM32F401RE board.
3. Open a serial terminal at 115200 baud on the USART2 TX line.
4. Observe magnetic field and temperature readings printed every second.

---

## Notes

- This example uses polling mode to read sensor data without interrupts.
- UART output uses blocking writes.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications.
- Panic behavior is set to halt on panic using `panic_halt`.

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [LIS2MDL Datasheet](https://www.st.com/resource/en/datasheet/lis2mdl.pdf)
- [stm32f4xx-hal Rust crate](https://docs.rs/stm32f4xx-hal)

---

*This README provides a detailed explanation of the embedded Rust program for magnetometer data acquisition on STM32F401RE using the LIS2MDL sensor.*
