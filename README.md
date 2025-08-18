# lis2mdl-rs
[![Crates.io][crates-badge]][crates-url]
[![BSD 3-Clause licensed][bsd-badge]][bsd-url]

[crates-badge]: https://img.shields.io/crates/v/lis2mdl-rs
[crates-url]: https://crates.io/crates/lis2mdl-rs
[bsd-badge]: https://img.shields.io/crates/l/lis2mdl-rs
[bsd-url]: https://opensource.org/licenses/BSD-3-Clause

Provides a platform-agnostic, no_std-compatible driver for the ST LIS2MDL sensor, supporting both I2C and SPI communication interfaces.

## Sensor Overview

The LIS2MDL is an ultra-low-power, highperformance 3-axis digital magnetic
sensor.

The LIS2MDL has a magnetic field dynamic
range of ±50 gauss.

The LIS2MDL includes an I2C serial bus interface
that supports standard, fast mode, fast mode
plus, and high-speed (100 kHz, 400 kHz,
1 MHz, and 3.4 MHz) and an SPI serial standard
interface.

The device can be configured to generate an
interrupt signal for magnetic field detection.

The LIS2MDL is available in a plastic land grid
array package (LGA) and is guaranteed to
operate over an extended temperature range
from -40 °C to +85 °C.

For more info, please visit the device page at [https://www.st.com/en/mems-and-sensors/lis2mdl.html](https://www.st.com/en/mems-and-sensors/lis2mdl.html)

## Installation

Add the driver to your `Cargo.toml` dependencies:

```toml
[dependencies]
lis2mdl-rs = "0.1.0"
```

Or, add it directly from the terminal:

```sh
cargo add lis2mdl-rs
```

## Usage

Include the crate and its prelude
```rust
use lis2mdl_rs as lis2mdl;
use lis2mdl::*;
use lis2mdl::prelude::*;
```

### Create an instance

Create an instance of the driver with the `new_<bus>` associated function, by passing an I2C (`embedded_hal::i2c::I2c`) instance and I2C address, or an SPI (`embedded_hal::spi::SpiDevice`) instance.

An example with I2C:

```rust
let mut sensor = Lis2mdl::new_i2c(i2c, I2CAddress::I2cAdd);
```

### Check "Who Am I" Register

This step ensures correct communication with the sensor. It returns a unique ID to verify the sensor's identity.

```rust
let whoami = sensor.device_id_get().unwrap();
if whoami != ID {
    panic!("Invalid sensor ID");
}
```

### Configure

See details in specific examples; the following are common api calls:

```rust
// Restore default configuration
sensor.reset_set(1).unwrap();

// Wait for reset to complete
while sensor.reset_get().unwrap() != 0 {}

// Enable Block Data Update
sensor.block_data_update_set(1).unwrap();

// Set Output Data Rate
sensor.data_rate_set(Odr::_10hz).unwrap();

// Set / Reset sensor mode
sensor
    .set_rst_mode_set(SetRst::SensOffCancEveryOdr)
    .unwrap();

// Enable temperature compensation
sensor.offset_temp_comp_set(1).unwrap();

// Set device in continuous mode
sensor.operating_mode_set(Md::ContinuousMode).unwrap();
```

## License

Distributed under the BSD-3 Clause license.

More Information: [http://www.st.com](http://st.com/MEMS).

**Copyright (C) 2025 STMicroelectronics**