use embedded_hal_mock::eh1::{
    spi::{Mock as SpiMock, Transaction},
    delay::{CheckedDelay, Transaction as DelayTransaction}
};
use rstest::rstest;

// Replace with your actual driver import
use lis2mdl_rs::{Lis2mdl, prelude::*};

mod common;
use common::*;

/// Test WHO_AM_I register over SPI
#[test]
fn test_device_id_get() {
    let spi_expect = [
        Transaction::transaction_start(),
        Transaction::write(registers::WHO_AM_I | 0x80),
        Transaction::read(0x40),
        Transaction::transaction_end()
    ];
    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());
    let id = sensor.device_id_get().unwrap();
    assert_eq!(id, 0x40);

    spi.done();
    delay.done();
}

#[rstest]
#[case(0x1095, 0x6109, 0x2321)]
#[case(0x4732, 0x2405, 0x0593)]
fn test_mag_user_offset(#[case] offset_x: i16, #[case] offset_y: i16, #[case] offset_z: i16) {
    // Example offset values
    let offsets = [offset_x, offset_y, offset_z];

    // Expected SPI transactions for writing offsets
    let spi_expect = [
        Transaction::transaction_start(),
        Transaction::write_vec(vec![
            registers::OFFSET_X_REG_L,
            // Write X offset (low byte, high byte)
            (offsets[0] & 0xFF) as u8,
            (offsets[0] >> 8) as u8,
            // Write Y offset (low byte, high byte)
            (offsets[1] & 0xFF) as u8,
            (offsets[1] >> 8) as u8,
            // Write Z offset (low byte, high byte)
            (offsets[2] & 0xFF) as u8,
            (offsets[2] >> 8) as u8
        ]),
        Transaction::transaction_end(),
        Transaction::transaction_start(),
        Transaction::write(registers::OFFSET_X_REG_L | 0x80),
        Transaction::read_vec(vec![
            // Read X offset (low byte, high byte)
            (offsets[0] & 0xFF) as u8,
            (offsets[0] >> 8) as u8,
            // Read Y offset (low byte, high byte)
            (offsets[1] & 0xFF) as u8,
            (offsets[1] >> 8) as u8,
            // Read Z offset (low byte, high byte)
            (offsets[2] & 0xFF) as u8,
            (offsets[2] >> 8) as u8]),
        Transaction::transaction_end()
    ];

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());
    let result = sensor.mag_user_offset_set(&offsets);
    assert!(result.is_ok());
    let value_get = sensor.mag_user_offset_get().unwrap();
    assert_eq!(value_get, offsets);

    spi.done();
    delay.done();
}

#[rstest]
#[case(Md::ContinuousMode, 0x00)]
#[case(Md::SingleTrigger,  0x01)]
#[case(Md::PowerDown,      0x03)]
fn test_operating_mode(#[case] mode: Md, #[case] mode_val: u8) {
    use registers::CFG_REG_A;

    // test CFG_REG_A default value
    let default_cfg_reg_a = CfgRegA::default().into_bits();

    let spi_expect = [
        // Set: read current register
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_A | 0x80),
        Transaction::read(default_cfg_reg_a),
        Transaction::transaction_end(),
        // Set: write new register value
        Transaction::transaction_start(),
        Transaction::write_vec(vec![CFG_REG_A, default_cfg_reg_a & mode_val]),
        Transaction::transaction_end(),
        // Get: read register value (should return the value we just wrote)
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_A | 0x80),
        Transaction::read(default_cfg_reg_a & mode_val),
        Transaction::transaction_end(),
    ];

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());
    // Set mode
    let result = sensor.operating_mode_set(mode);
    assert!(result.is_ok());

    // Get mode
    let mode_get = sensor.operating_mode_get().unwrap();
    assert_eq!(mode_get, mode);

    spi.done();
    delay.done();
}

#[rstest]
#[case(Odr::_10hz, 0x00)]
#[case(Odr::_20hz, 0x01)]
#[case(Odr::_50hz, 0x02)]
#[case(Odr::_100hz,0x03)]
fn test_data_rate(#[case] odr: Odr, #[case] reg_value: u8) {
    use registers::CFG_REG_A;

    let default_cfg_reg_a = CfgRegA::default().into_bits();
    let reg_value = default_cfg_reg_a | (reg_value << 2);

    let spi_expect = [
        // Set: read current register
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_A | 0x80),
        Transaction::read(default_cfg_reg_a),
        Transaction::transaction_end(),
        // Set: write new register value
        Transaction::transaction_start(),
        Transaction::write_vec(vec![CFG_REG_A, reg_value]),
        Transaction::transaction_end(),
        // Get: read register value (should return the value we just wrote)
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_A | 0x80),
        Transaction::read(reg_value),
        Transaction::transaction_end(),
    ];

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());
    // Set mode
    let result = sensor.data_rate_set(odr);
    assert!(result.is_ok());

    // Get mode
    let odr_get = sensor.data_rate_get().unwrap();
    assert_eq!(odr_get, odr);

    spi.done();
    delay.done();
}

#[rstest]
#[case(PowerMode::HighResolution, 0x00)]
#[case(PowerMode::LowPower,       0x01)]
fn test_power_mode(#[case] power_mode: PowerMode, #[case] reg_value: u8) {
    use registers::CFG_REG_A;

    let default_cfg_reg_a = CfgRegA::default().into_bits();
    let reg_value = default_cfg_reg_a | (reg_value << 4);

    let spi_expect = [
        // Set: read current register
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_A | 0x80),
        Transaction::read(default_cfg_reg_a),
        Transaction::transaction_end(),
        // Set: write new register value
        Transaction::transaction_start(),
        Transaction::write_vec(vec![CFG_REG_A, reg_value]),
        Transaction::transaction_end(),
        // Get: read register value (should return the value we just wrote)
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_A | 0x80),
        Transaction::read(reg_value),
        Transaction::transaction_end(),
    ];

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());
    // Set mode
    let result = sensor.power_mode_set(power_mode);
    assert!(result.is_ok());

    // Get mode
    let power_mode_get = sensor.power_mode_get().unwrap();
    assert_eq!(power_mode_get, power_mode);

    spi.done();
    delay.done();
}

#[rstest]
#[case(0, 0x00)]
#[case(1, 0x01)]
fn test_offset_temp_comp(#[case] value: u8, #[case] reg_value: u8) {
    use registers::CFG_REG_A;

    let default_cfg_reg_a = CfgRegA::default().into_bits();
    let reg_value = default_cfg_reg_a | (reg_value << 7);

    let spi_expect = [
        // Set: read current register
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_A | 0x80),
        Transaction::read(default_cfg_reg_a),
        Transaction::transaction_end(),
        // Set: write new register value
        Transaction::transaction_start(),
        Transaction::write_vec(vec![CFG_REG_A, reg_value]),
        Transaction::transaction_end(),
        // Get: read register value (should return the value we just wrote)
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_A | 0x80),
        Transaction::read(reg_value),
        Transaction::transaction_end(),
    ];

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());
    // Set mode
    let result = sensor.offset_temp_comp_set(value);
    assert!(result.is_ok());

    // Get mode
    let offset_temp_get = sensor.offset_temp_comp_get().unwrap();
    assert_eq!(offset_temp_get, value);

    spi.done();
    delay.done();
}

#[rstest]
#[case(LowPassFilter::OdrDiv2, 0x00)]
#[case(LowPassFilter::OdrDiv4, 0x01)]
fn test_low_pass_bandwidth(#[case] filter: LowPassFilter, #[case] reg_value: u8) {
    use registers::CFG_REG_B;

    let default_cfg_reg_b = CfgRegB::default().into_bits();
    let reg_value = default_cfg_reg_b | (reg_value << 0);

    let spi_expect = [
        // Set: read current register
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_B | 0x80),
        Transaction::read(default_cfg_reg_b),
        Transaction::transaction_end(),
        // Set: write new register value
        Transaction::transaction_start(),
        Transaction::write_vec(vec![CFG_REG_B, reg_value]),
        Transaction::transaction_end(),
        // Get: read register value (should return the value we just wrote)
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_B | 0x80),
        Transaction::read(reg_value),
        Transaction::transaction_end(),
    ];

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());
    // Set mode
    let result = sensor.low_pass_bandwidth_set(filter);
    assert!(result.is_ok());

    // Get mode
    let low_pass_filter_get = sensor.low_pass_bandwidth_get().unwrap();
    assert_eq!(low_pass_filter_get, filter);

    spi.done();
    delay.done();
}

#[rstest]
#[case(SetRst::SetSensOdrDiv63,      0x00, 0x00)]
#[case(SetRst::SensOffCancEveryOdr,  0x01, 0x02)]
#[case(SetRst::SetSensOnlyAtPowerOn, 0x02, 0x04)]
fn test_set_rst_mode(#[case] set_rst: SetRst, #[case] _reg_value: u8, #[case] expected_reg: u8) {
    use registers::CFG_REG_B;

    let default_cfg_reg_b = CfgRegB::default().into_bits();

    let spi_expect = [
        // Set: read current register
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_B | 0x80),
        Transaction::read(default_cfg_reg_b),
        Transaction::transaction_end(),
        // Set: write new register value
        Transaction::transaction_start(),
        Transaction::write_vec(vec![CFG_REG_B, expected_reg]),
        Transaction::transaction_end(),
        // Get: read register value (should return the value we just wrote)
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_B | 0x80),
        Transaction::read(expected_reg),
        Transaction::transaction_end(),
    ];

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());
    // Set mode
    let result = sensor.set_rst_mode_set(set_rst);
    assert!(result.is_ok());

    // Get mode
    let set_rst_get = sensor.set_rst_mode_get().unwrap();
    assert_eq!(set_rst_get, set_rst);

    spi.done();
    delay.done();
}

/// Should generate errors, because enabling offset cancellation in single measurement mode_val
/// require to have OFF_CANC bit set as well
#[rstest]
#[case(0x00, true)]
#[case(0x01, false)]
fn test_set_sensor_single_error(#[case] value: u8, #[case] should_be_ok: bool) {
    use registers::CFG_REG_B;

    let default_cfg_reg_b = CfgRegB::default().into_bits();

    let spi_expect = [
        // Set: read current register
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_B | 0x80),
        Transaction::read(default_cfg_reg_b),
        Transaction::transaction_end(),
        // Write: only if should_be_ok is true
        Transaction::transaction_start(),
        Transaction::write_vec(vec![CFG_REG_B, 0x00]), // OFF_CANC bit
        Transaction::transaction_end(),
    ];

    let spi_expect = if should_be_ok { spi_expect.to_vec() } else { spi_expect[0..4].to_vec() };

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());
    // Expected error: OFF_CANC bit not set when trying to set OFF_CANC_ONE_SHOT to 1
    let result = sensor.set_rst_sensor_single_set(value);
    assert_eq!(result.is_ok(), should_be_ok);

    spi.done();
    delay.done();
}

/// Enabling offset cancellation in single measurement mode require to have OFF_CANC bit set as
/// well
#[rstest]
#[case(0x00, 0x02)]
#[case(0x01, 0x12)]
fn test_set_sensor_single(#[case] value: u8, #[case] expected_reg: u8) {
    use registers::CFG_REG_B;

    let default_cfg_reg_b = CfgRegB::default().into_bits();

    let spi_expect = [
        // Set: set_rst_mode_set
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_B | 0x80),
        Transaction::read(default_cfg_reg_b),
        Transaction::transaction_end(),
        Transaction::transaction_start(),
        Transaction::write_vec(vec![CFG_REG_B, 0x02]), // OFF_CANC bit
        Transaction::transaction_end(),

        // Set: set_rst_sensor_single_set
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_B | 0x80),
        Transaction::read(0x02),
        Transaction::transaction_end(),
        Transaction::transaction_start(),
        Transaction::write_vec(vec![CFG_REG_B, expected_reg]),
        Transaction::transaction_end(),

        // Get: set_rst_sensor_single_get
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_B | 0x80),
        Transaction::read(expected_reg),
        Transaction::transaction_end(),
    ];

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());
    let result = sensor.set_rst_mode_set(SetRst::SensOffCancEveryOdr);
    assert!(result.is_ok());
    let result = sensor.set_rst_sensor_single_set(value);
    assert!(result.is_ok());

    // Get mode
    let value_get = sensor.set_rst_sensor_single_get().unwrap();
    assert_eq!(value_get, value);

    spi.done();
    delay.done();
}

#[rstest]
#[case(0x00, 0x00)]
#[case(0x01, 0x10)]
fn test_block_data_update(#[case] value: u8, #[case] expected_reg: u8) {
    use registers::CFG_REG_C;

    let default_cfg_reg_c = CfgRegC::default().into_bits();

    let spi_expect = [
        // Set: read current register
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_C | 0x80),
        Transaction::read(default_cfg_reg_c),
        Transaction::transaction_end(),
        // Set: write new register value
        Transaction::transaction_start(),
        Transaction::write_vec(vec![CFG_REG_C, expected_reg]),
        Transaction::transaction_end(),
        // Get: read register value (should return the value we just wrote)
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_C | 0x80),
        Transaction::read(expected_reg),
        Transaction::transaction_end(),
    ];

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());
    // Set mode
    let result = sensor.block_data_update_set(value);
    assert!(result.is_ok());

    // Get mode
    let value_get = sensor.block_data_update_get().unwrap();
    assert_eq!(value_get, value);

    spi.done();
    delay.done();
}

#[rstest]
#[case(0x09, 0xA0)]
#[case(0x08, 0x90)]
#[case(0x0A, 0x80)]
fn test_status_mag_ovr(#[case] expected_reg_for_mag: u8, #[case] expected_reg_for_data_ovr: u8) {
    use registers::STATUS_REG;

    let default_status = StatusReg::default().into_bits();

    let spi_expect = [
        // Read: Assume no event
        Transaction::transaction_start(),
        Transaction::write(STATUS_REG | 0x80),
        Transaction::read(default_status),
        Transaction::transaction_end(),
        // Read: Assume mag data ready event
        Transaction::transaction_start(),
        Transaction::write(STATUS_REG | 0x80),
        Transaction::read(expected_reg_for_mag),
        Transaction::transaction_end(),
        // Read: Assume data_ovr event
        Transaction::transaction_start(),
        Transaction::write(STATUS_REG | 0x80),
        Transaction::read(expected_reg_for_data_ovr),
        Transaction::transaction_end(),
    ];

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());

    // No event
    let result = sensor.mag_data_ready_get();
    assert!(result.is_ok());
    assert_eq!(result.unwrap(), 0);

    // Mag data ready event
    let result = sensor.mag_data_ready_get();
    assert!(result.is_ok());
    assert_eq!(result.unwrap(), 1);

    // Data_ovr event
    let result = sensor.mag_data_ovr_get();
    assert!(result.is_ok());
    assert_eq!(result.unwrap(), 1);

    spi.done();
    delay.done();
}

#[test]
fn test_status() {
    use registers::STATUS_REG;

    let default_status = StatusReg::default().into_bits();

    let spi_expect = [
        // Read: Assume to read status register
        Transaction::transaction_start(),
        Transaction::write(STATUS_REG | 0x80),
        Transaction::read(default_status | 0x9C),
        Transaction::transaction_end(),
    ];

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());

    // status_get
    let result = sensor.status_get();
    assert!(result.is_ok());
    let result = result.unwrap();
    assert_eq!(result.zyxor(), 1);
    assert_eq!(result.xor(), 1);
    assert_eq!(result.yor(), 0);
    assert_eq!(result.zor(), 0);
    assert_eq!(result.zyxda(), 1);
    assert_eq!(result.xda(), 0);
    assert_eq!(result.yda(), 0);
    assert_eq!(result.zda(), 1);

    spi.done();
    delay.done();
}

// Execute the test with differents 'no update' reads
#[rstest]
#[case(0, true)]
#[case(1, true)]
#[case(2, true)]
#[case(3, true)]
#[case(4, false)]
fn test_sw_reset_set(#[case] no_update_times: u8, #[case] should_be_ok: bool) {
    use registers::CFG_REG_A;

    let default_cfg_reg_a = CfgRegA::default().into_bits();

    /* 1. Set SOFT_RST without any read */
    let mut spi_expect = vec![
        // Write SOFT_RST bit of the CFG_REG_A
        Transaction::transaction_start(),
        Transaction::write_vec(vec![CFG_REG_A, 0x20 | default_cfg_reg_a]),
        Transaction::transaction_end(),
    ];
    let mut delay_expect = vec![];

    /* 2. Poll SOFT_RST, here every output is: "no update" */
    for _ in 0..no_update_times {
        // Poll first time with no update
        spi_expect.push(Transaction::transaction_start());
        spi_expect.push(Transaction::write(CFG_REG_A | 0x80));
        spi_expect.push(Transaction::read(0x20 | default_cfg_reg_a));
        spi_expect.push(Transaction::transaction_end());
        delay_expect.push(DelayTransaction::delay_us(5));
    }

    // driver will read bit cleared only if no error expected (attempts < 4)
    if should_be_ok {
        // Poll second time with bit cleared
        spi_expect.push(Transaction::transaction_start());
        spi_expect.push(Transaction::write(CFG_REG_A | 0x80));
        spi_expect.push(Transaction::read(0x00 | default_cfg_reg_a));
        spi_expect.push(Transaction::transaction_end());
    } else {
        // no wait for the last failed read
        delay_expect.pop();
    }

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&delay_expect);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());

    let result = sensor.sw_reset();
    assert_eq!(result.is_ok(), should_be_ok);

    spi.done();
    delay.done();
}

#[test]
fn test_reboot() {
    use registers::CFG_REG_A;

    let default_cfg_reg_a = CfgRegA::default().into_bits();
    let reg_value = default_cfg_reg_a | (0x40);

    let spi_expect = [
        // Read because reboot do not erase CfgRegA memory
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_A | 0x80),
        Transaction::read(default_cfg_reg_a),
        Transaction::transaction_end(),
        // Write boot bit then wait
        Transaction::transaction_start(),
        Transaction::write_vec(vec![CFG_REG_A, reg_value]),
        Transaction::transaction_end(),
    ];

    let delay_expected = vec![
        DelayTransaction::delay_ms(20)
    ];

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&delay_expected);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());

    let result = sensor.reboot();
    assert!(result.is_ok());

    spi.done();
    delay.done();
}

#[rstest]
#[case(0x00, 0x00)]
#[case(0x01, 0x02)]
fn test_self_test(#[case] value: u8, #[case] expected_reg: u8) {
    use registers::CFG_REG_C;

    let default_cfg_reg_c = CfgRegC::default().into_bits();

    let spi_expect = [
        // Set: read current register
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_C | 0x80),
        Transaction::read(default_cfg_reg_c),
        Transaction::transaction_end(),
        // Set: write new register value
        Transaction::transaction_start(),
        Transaction::write_vec(vec![CFG_REG_C, expected_reg]),
        Transaction::transaction_end(),
        // Get: read register value (should return the value we just wrote)
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_C | 0x80),
        Transaction::read(expected_reg),
        Transaction::transaction_end(),
    ];

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());
    // Set mode
    let result = sensor.self_test_set(value);
    assert!(result.is_ok());

    // Get mode
    let value_get = sensor.self_test_get().unwrap();
    assert_eq!(value_get, value);

    spi.done();
    delay.done();
}

#[rstest]
#[case(Ble::LsbAtLowAdd, 0x00)]
#[case(Ble::MsbAtLowAdd, 0x08)]
fn test_data_format(#[case] value: Ble, #[case] expected_reg: u8) {
    use registers::CFG_REG_C;

    let default_cfg_reg_c = CfgRegC::default().into_bits();

    let spi_expect = [
        // Set: read current register
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_C | 0x80),
        Transaction::read(default_cfg_reg_c),
        Transaction::transaction_end(),
        // Set: write new register value
        Transaction::transaction_start(),
        Transaction::write_vec(vec![CFG_REG_C, expected_reg]),
        Transaction::transaction_end(),
        // Get: read register value (should return the value we just wrote)
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_C | 0x80),
        Transaction::read(expected_reg),
        Transaction::transaction_end(),
    ];

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());
    // Set data format
    let result = sensor.data_format_set(value);
    assert!(result.is_ok());

    // Get data format
    let value_get = sensor.data_format_get().unwrap();
    assert_eq!(value_get, value);

    spi.done();
    delay.done();
}

#[rstest]
#[case(IntOnDataoff::CheckAfter, 0x08)]
#[case(IntOnDataoff::CheckBefore, 0x00)]
fn test_offset_int_conf(#[case] value: IntOnDataoff, #[case] expected_reg: u8) {
    use registers::CFG_REG_B;

    let default_cfg_reg_b = CfgRegB::default().into_bits();

    let spi_expect = [
        // Set: read current register
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_B | 0x80),
        Transaction::read(default_cfg_reg_b),
        Transaction::transaction_end(),
        // Set: write new register value
        Transaction::transaction_start(),
        Transaction::write_vec(vec![CFG_REG_B, expected_reg]),
        Transaction::transaction_end(),
        // Get: read register value (should return the value we just wrote)
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_B | 0x80),
        Transaction::read(expected_reg),
        Transaction::transaction_end(),
    ];

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());
    // Set offset interrupt configuration
    let result = sensor.offset_int_conf_set(value);
    assert!(result.is_ok());

    // Get offset interrupt configuration
    let value_get = sensor.offset_int_conf_get().unwrap();
    assert_eq!(value_get, value);

    spi.done();
    delay.done();
}

#[rstest]
#[case(0x00, 0x00)]
#[case(0x01, 0x01)]
fn test_drdy_on_pin(#[case] value: u8, #[case] expected_reg: u8) {
    use registers::CFG_REG_C;

    let default_cfg_reg_c = CfgRegC::default().into_bits();

    let spi_expect = [
        // Set: read current register
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_C | 0x80),
        Transaction::read(default_cfg_reg_c),
        Transaction::transaction_end(),
        // Set: write new register value
        Transaction::transaction_start(),
        Transaction::write_vec(vec![CFG_REG_C, expected_reg]),
        Transaction::transaction_end(),
        // Get: read register value (should return the value we just wrote)
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_C | 0x80),
        Transaction::read(expected_reg),
        Transaction::transaction_end(),
    ];

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());
    // Set mode
    let result = sensor.drdy_on_pin_set(value);
    assert!(result.is_ok());

    // Get mode
    let value_get = sensor.drdy_on_pin_get().unwrap();
    assert_eq!(value_get, value);

    spi.done();
    delay.done();
}

#[rstest]
#[case(0x00, 0x00)]
#[case(0x01, 0x40)]
fn test_int_on_pin(#[case] value: u8, #[case] expected_reg: u8) {
    use registers::CFG_REG_C;

    let default_cfg_reg_c = CfgRegC::default().into_bits();

    let spi_expect = [
        // Set: read current register
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_C | 0x80),
        Transaction::read(default_cfg_reg_c),
        Transaction::transaction_end(),
        // Set: write new register value
        Transaction::transaction_start(),
        Transaction::write_vec(vec![CFG_REG_C, expected_reg]),
        Transaction::transaction_end(),
        // Get: read register value (should return the value we just wrote)
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_C | 0x80),
        Transaction::read(expected_reg),
        Transaction::transaction_end(),
    ];

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());
    // Set mode
    let result = sensor.int_on_pin_set(value);
    assert!(result.is_ok());

    // Get mode
    let value_get = sensor.int_on_pin_get().unwrap();
    assert_eq!(value_get, value);

    spi.done();
    delay.done();
}

#[rstest]
#[case(IntCrtlReg::default(),       0xE0)]
#[case(IntCrtlReg::from_bits(0xA2), 0xA2)]
fn test_int_gen_conf(#[case] value: IntCrtlReg, #[case] expected_reg: u8) {
    use registers::INT_CRTL_REG;

    let spi_expect = [
        // Set: write new register value
        Transaction::transaction_start(),
        Transaction::write_vec(vec![INT_CRTL_REG, expected_reg]),
        Transaction::transaction_end(),
        // Get: read register value (should return the value we just wrote)
        Transaction::transaction_start(),
        Transaction::write(INT_CRTL_REG | 0x80),
        Transaction::read(expected_reg),
        Transaction::transaction_end(),
    ];

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());
    // Set interrupt generator configuration
    let result = sensor.int_gen_conf_set(&value);
    assert!(result.is_ok());

    // Get interrupt generator configuration
    let value_get = sensor.int_gen_conf_get().unwrap();
    assert_eq!(value_get.into_bits(), value.into_bits());

    spi.done();
    delay.done();
}

#[rstest]
#[case(0x0000, 0x0000)]
#[case(0x1234, 0x1234)]
#[case(0xF213, 0xF213)]
fn test_int_gen_threshold(#[case] value: u16, #[case] expected_reg: u16) {
    use registers::INT_THS_L_REG;

    let spi_expect = [
        // Set: write new register value (assuming 16-bit register, split into two bytes)
        Transaction::transaction_start(),
        Transaction::write_vec(vec![
            INT_THS_L_REG,
            (expected_reg & 0xFF) as u8,
            (expected_reg >> 8) as u8
        ]),
        Transaction::transaction_end(),
        // Get: read register value (should return the value we just wrote)
        Transaction::transaction_start(),
        Transaction::write(INT_THS_L_REG | 0x80),
        Transaction::read_vec(vec![(expected_reg & 0xFF) as u8, (expected_reg >> 8) as u8]),
        Transaction::transaction_end(),
    ];

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());
    // Set threshold
    let result = sensor.int_gen_threshold_set(value);
    assert!(result.is_ok());

    // Get threshold
    let value_get = sensor.int_gen_threshold_get().unwrap();
    assert_eq!(value_get, value);

    spi.done();
    delay.done();
}

#[rstest]
#[case(Sim::Spi3Wire, 0x00)]
#[case(Sim::Spi4Wire, 0x04)]
fn test_spi_mode(#[case] value: Sim, #[case] expected_reg: u8) {
    use registers::CFG_REG_C;

    let default_cfg_reg_c = CfgRegC::default().into_bits();

    let spi_expect = [
        // Set: read current register
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_C | 0x80),
        Transaction::read(default_cfg_reg_c),
        Transaction::transaction_end(),
        // Set: write new register value
        Transaction::transaction_start(),
        Transaction::write_vec(vec![CFG_REG_C, expected_reg]),
        Transaction::transaction_end(),
        // Get: read register value (should return the value we just wrote)
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_C | 0x80),
        Transaction::read(expected_reg),
        Transaction::transaction_end(),
    ];

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());
    // Set SPI mode
    let result = sensor.spi_mode_set(value);
    assert!(result.is_ok());

    // Get SPI mode
    let value_get = sensor.spi_mode_get().unwrap();
    assert_eq!(value_get, value);

    spi.done();
    delay.done();
}

#[rstest]
#[case(I2cSwitch::I2cEnable,  0x00)]
#[case(I2cSwitch::I2cDisable, 0x20)]
fn test_i2c_interface(#[case] value: I2cSwitch, #[case] expected_reg: u8) {
    use registers::CFG_REG_C;

    let default_cfg_reg_c = CfgRegC::default().into_bits();

    let spi_expect = [
        // Set: read current register
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_C | 0x80),
        Transaction::read(default_cfg_reg_c),
        Transaction::transaction_end(),
        // Set: write new register value
        Transaction::transaction_start(),
        Transaction::write_vec(vec![CFG_REG_C, expected_reg]),
        Transaction::transaction_end(),
        // Get: read register value (should return the value we just wrote)
        Transaction::transaction_start(),
        Transaction::write(CFG_REG_C | 0x80),
        Transaction::read(expected_reg),
        Transaction::transaction_end(),
    ];

    let mut spi = SpiMock::new(&spi_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_spi(spi.clone(), delay.clone());
    // Set I2C interface
    let result = sensor.i2c_interface_set(value);
    assert!(result.is_ok());

    // Get I2C interface
    let value_get = sensor.i2c_interface_get().unwrap();
    assert_eq!(value_get, value);

    spi.done();
    delay.done();
}
