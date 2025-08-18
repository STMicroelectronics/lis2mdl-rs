#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::NoDma;
use embassy_stm32::i2c::{self, Config as I2cConfig, I2c};
use embassy_stm32::peripherals::{self, USART2};
use embassy_stm32::time::khz;
use embassy_stm32::usart::{
    BufferedInterruptHandler, Config as UsartConfig, DataBits, Parity, UartTx,
};
use embassy_time::Delay;
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

use lis2mdl_rs::prelude::*;
use lis2mdl_rs::*;

#[defmt::panic_handler]
fn panic() -> ! {
    core::panic!("panic via `defmt::panic!`")
}

bind_interrupts!(struct Irqs {
    USART2 => BufferedInterruptHandler<USART2>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut usart_config: UsartConfig = UsartConfig::default();
    usart_config.baudrate = 115200;
    usart_config.data_bits = DataBits::DataBits8;
    usart_config.parity = Parity::ParityNone;

    let mut tx: UartTx<_> = UartTx::new(p.USART2, p.PA2, NoDma, usart_config).unwrap();

    let i2c: I2c<_> = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        NoDma,
        NoDma,
        khz(100),
        I2cConfig::default(),
    );

    let mut delay = Delay;

    delay.delay_ms(5_u32);

    let mut msg: String<64> = String::new();

    let mut sensor = Lis2mdl::new_i2c(i2c, I2CAddress::I2cAdd);

    // Check device ID
    match sensor.device_id_get() {
        Ok(value) => writeln!(&mut msg, "LIS2MDL id: {:#02x}", value).unwrap(),
        Err(e) => writeln!(&mut msg, "Error in reading id: {:?}", e).unwrap(),
    }
    let _ = tx.blocking_write(msg.as_bytes());
    msg.clear();

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
                        &mut msg,
                        "Magnetic field [mG]: {:.2}\t{:.2}\t{:.2}",
                        magnetic_mg[0], magnetic_mg[1], magnetic_mg[2]
                    )
                    .unwrap();
                }
                Err(e) => writeln!(&mut msg, "Error in reading magnetic data: {:?}", e).unwrap(),
            }
            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();

            // Read temperature data
            match sensor.temperature_raw_get() {
                Ok(data_raw_temperature) => {
                    let temperature_deg_c = from_lsb_to_celsius(data_raw_temperature);
                    writeln!(&mut msg, "Temperature [degC]: {:.2}", temperature_deg_c).unwrap();
                }
                Err(e) => writeln!(&mut msg, "Error in reading temperature: {:?}", e).unwrap(),
            }
            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();
        }
        delay.delay_ms(1000_u32);
    }
}
