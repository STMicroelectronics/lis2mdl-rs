use embedded_hal_mock::eh1::{
    i2c::{Mock, Transaction},
    delay::{CheckedDelay, Transaction as DelayTransaction}
};
use lis2mdl_rs::{Lis2mdl, I2CAddress, prelude::*};
use rstest::rstest;

const DEVICE_ADDR: I2CAddress = I2CAddress::I2cAdd;

mod common;
use common::{*};

// Representation at the bus level of how latching works,
// how registers are reset, and under which conditions.
#[test]
fn int_source_reg_latched() {
    use registers::{INT_SOURCE_REG, INT_CRTL_REG};

    let int_ctrl_reg_default = IntCrtlReg::default().into_bits();

    // Reading IntSourceGet with latched enable autoclear of the register content
    let i2c_expect = [
        // First read: bits are set
        Transaction::write_read(DEVICE_ADDR as u8, vec![INT_SOURCE_REG], vec![0b101000]),
        Transaction::write_read(DEVICE_ADDR as u8, vec![INT_SOURCE_REG], vec![0b101000]),
        //Transaction::write_read(DEVICE_ADDR as u8, vec![INT_CRTL_REG], vec![int_ctrl_reg_default]),
        Transaction::write(DEVICE_ADDR as u8, vec![INT_CRTL_REG, int_ctrl_reg_default | 0x02]),
        Transaction::write_read(DEVICE_ADDR as u8, vec![INT_SOURCE_REG], vec![0b101000]),
        Transaction::write_read(DEVICE_ADDR as u8, vec![INT_SOURCE_REG], vec![0b000000]),
    ];

    let mut i2c = Mock::new(&i2c_expect);
    let mut delay = CheckedDelay::new(&[]);

    let mut sensor = Lis2mdl::new_i2c(i2c.clone(), DEVICE_ADDR, delay.clone());

    let res = sensor.int_gen_source_get().unwrap();
    assert!(res.into_bits() != 0);

    // Second read do not clear the register
    let res = sensor.int_gen_source_get().unwrap();
    assert!(res.into_bits() != 0);


    /* Latched mode to clear the register after a read */
    let mut ctrl = IntCrtlReg::default();
    ctrl.set_iel(1);
    sensor.int_gen_conf_set(&ctrl).unwrap();

    let res = sensor.int_gen_source_get().unwrap();
    assert!(res.into_bits() != 0);

    // Second read, with Latched mode => Clears the register
    let res = sensor.int_gen_source_get().unwrap();
    assert!(res.into_bits() == 0);

    i2c.done();
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
    let mut i2c_expect = vec![
        // Write SOFT_RST bit of the CFG_REG_A
        Transaction::write(DEVICE_ADDR as u8, vec![CFG_REG_A, 0x20 | default_cfg_reg_a]),
    ];
    let mut delay_expect = vec![];

    /* 2. Poll SOFT_RST, here every output is: "no update" */
    for _ in 0..no_update_times {
        // Poll first time with no update
        i2c_expect.push(Transaction::write_read(
            DEVICE_ADDR as u8,
            vec![CFG_REG_A],
            vec![0x20 | default_cfg_reg_a],
        ));
        delay_expect.push(DelayTransaction::delay_us(5));
    }

    // driver will read bit cleared only if no error expected (attempts < 4)
    if should_be_ok {
        // Poll second time with bit cleared
        i2c_expect.push(Transaction::write_read(
            DEVICE_ADDR as u8,
            vec![CFG_REG_A],
            vec![0x00 | default_cfg_reg_a],
        ));
    } else {
        // no wait for the last failed read
        delay_expect.pop();
    }

    let mut i2c = Mock::new(&i2c_expect);
    let mut delay = CheckedDelay::new(&delay_expect);

    let mut sensor = Lis2mdl::new_i2c(i2c.clone(), DEVICE_ADDR, delay.clone());

    let result = sensor.sw_reset();
    assert_eq!(result.is_ok(), should_be_ok);

    i2c.done();
    delay.done();
}

#[test]
fn test_reboot() {
    use registers::CFG_REG_A;

    let default_cfg_reg_a = CfgRegA::default().into_bits();
    let reg_value = default_cfg_reg_a | (0x40);

    let i2c_expect = [
        // Read because reboot does not erase CfgRegA memory
        Transaction::write_read(DEVICE_ADDR as u8, vec![CFG_REG_A], vec![default_cfg_reg_a]),
        // Write boot bit then wait
        Transaction::write(DEVICE_ADDR as u8, vec![CFG_REG_A, reg_value]),
    ];

    let delay_expected = vec![
        DelayTransaction::delay_ms(20)
    ];

    let mut i2c = Mock::new(&i2c_expect);
    let mut delay = CheckedDelay::new(&delay_expected);

    let mut sensor = Lis2mdl::new_i2c(i2c.clone(), DEVICE_ADDR, delay.clone());

    let result = sensor.reboot();
    assert!(result.is_ok());

    i2c.done();
    delay.done();
}
