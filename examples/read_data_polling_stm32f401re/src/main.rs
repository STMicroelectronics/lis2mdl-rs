#![no_std]
#![no_main]
#![deny(unsafe_code)]

use core::fmt::Write;
use cortex_m_rt::entry;
use lis2mdl_rs::prelude::*;
use lis2mdl_rs::*;
use panic_halt as _;
use stm32f4xx_hal::{
    i2c::{DutyCycle, I2c, Mode},
    pac,
    prelude::*,
    serial::{config::Config, Serial},
};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();

    let mut delay = cp.SYST.delay(&clocks);

    let gpiob = dp.GPIOB.split();
    let gpioa = dp.GPIOA.split();

    let scl = gpiob.pb8.into_alternate().set_open_drain();
    let sda = gpiob.pb9.into_alternate().set_open_drain();

    let i2c = I2c::new(
        dp.I2C1,
        (scl, sda),
        Mode::Fast {
            frequency: 400.kHz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        &clocks,
    );

    let tx_pin = gpioa.pa2.into_alternate();

    let mut tx = Serial::tx(
        dp.USART2,
        tx_pin,
        Config::default().baudrate(115_200.bps()),
        &clocks,
    )
    .unwrap();

    delay.delay_ms(5);

    let mut sensor = Lis2mdl::new_i2c(i2c, I2CAddress::I2cAdd);

    // Check device ID
    match sensor.device_id_get() {
        Ok(value) => writeln!(tx, "LIS2MDL id: {:#02x}", value).unwrap(),
        Err(e) => writeln!(tx, "Error in reading id: {:?}", e).unwrap(),
    }

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

    // Read samples in polling mode (no int)
    loop {
        // Read output only if new value is available
        if sensor.mag_data_ready_get().unwrap() != 0 {
            // Read magnetic field data
            match sensor.magnetic_raw_get() {
                Ok(data_raw_magnetic) => {
                    let magnetic_mg = [
                        from_lsb_to_mgauss(data_raw_magnetic[0]),
                        from_lsb_to_mgauss(data_raw_magnetic[1]),
                        from_lsb_to_mgauss(data_raw_magnetic[2]),
                    ];
                    writeln!(
                        tx,
                        "Magnetic field [mG]: {:.2}\t{:.2}\t{:.2}",
                        magnetic_mg[0], magnetic_mg[1], magnetic_mg[2]
                    )
                    .unwrap();
                }
                Err(e) => writeln!(tx, "Error in reading magnetic data: {:?}", e).unwrap(),
            }

            // Read temperature data
            match sensor.temperature_raw_get() {
                Ok(data_raw_temperature) => {
                    let temperature_deg_c = from_lsb_to_celsius(data_raw_temperature);
                    writeln!(tx, "Temperature [degC]: {:.2}", temperature_deg_c).unwrap();
                }
                Err(e) => writeln!(tx, "Error in reading temperature: {:?}", e).unwrap(),
            }
        }
        delay.delay_ms(1000_u32);
    }
}
