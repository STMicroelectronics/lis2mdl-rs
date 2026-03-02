use defmt::info;
use maybe_async::maybe_async;
use crate::*;

/*
* Fill magnetometer field offset (positive and negative values)
*
* The computation of the hard-iron distortion field should
* be performed by an external processor. After the computation
* of the hard iron-distortion field has been performed, the
* measured magnetic data can be compensated.
* These values act on the magnetic output data value in order
* to delete the environmental offset.
*/

const MAG_OFFSET: [i16; 3] = [
    0xF500u16 as i16, // OFFSET_X_REG
    0xF800u16 as i16, // OFFSET_Y_REG
    0xF400u16 as i16, // OFFSET_Z_REG
];

#[maybe_async]
pub async fn run<B, D, L>(bus: B, mut tx: L, mut delay: D, _irq: ()) -> !
where
    B: BusOperation,
    D: DelayNs + Clone,
    L: embedded_io::Write
{
    use lis2mdl::prelude::*;
    use lis2mdl::*;

    info!("Configuring the sensor");
    let mut sensor = Lis2mdl::new_bus(bus, delay.clone());

    // boot time
    delay.delay_ms(5).await;

    // Check device ID
    let id = sensor.device_id_get().await.unwrap();
    info!("Device ID: {:x}", id);
    if id != ID {
        info!("Unexpected device ID: {:x}", id);
        writeln!(tx, "Unexpected device ID: {:x}", id).unwrap();
        loop {}
    }

    // Restore default configuration
    sensor.sw_reset().await.unwrap();

    // Enable Block Data Update
    sensor.block_data_update_set(1).await.unwrap();

    // Set Output Data Rate
    sensor.data_rate_set(Odr::_10hz).await.unwrap();

    // Set / Reset sensor mode
    sensor
        .set_rst_mode_set(SetRst::SensOffCancEveryOdr)
        .await.unwrap();

    // Enable temperature compensation
    sensor.offset_temp_comp_set(1).await.unwrap();

    // Set device in continuous mode
    sensor.operating_mode_set(Md::ContinuousMode).await.unwrap();

    // Configure Mag offset and enable cancellation
    sensor.mag_user_offset_set(&MAG_OFFSET).await.unwrap();

    // Read samples in polling mode (no int)
    loop {
        // Read output only if new value is available
        if sensor.mag_data_ready_get().await.unwrap() != 0 {
            // Read magnetic field data
            match sensor.magnetic_raw_get().await {
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
            match sensor.temperature_raw_get().await {
                Ok(data_raw_temperature) => {
                    let temperature_deg_c = from_lsb_to_celsius(data_raw_temperature);
                    writeln!(tx, "Temperature [degC]: {:.2}", temperature_deg_c).unwrap();
                }
                Err(e) => writeln!(tx, "Error in reading temperature: {:?}", e).unwrap(),
            }
        }
        delay.delay_ms(1000_u32).await;
    }
}
