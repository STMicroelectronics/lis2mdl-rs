use lis2mdl_rs::prelude::*;

mod common;

#[test]
fn test_offset() {
    let offset = OffsetXYZ::default();

    assert_eq!(offset.x, 0);
    assert_eq!(offset.y, 0);
    assert_eq!(offset.z, 0);
}

#[test]
fn test_default_who_am_i() {
    let reg = WhoAmI::default().into_bits();
    assert_eq!(reg, 0b01000000);
}

#[test]
fn test_default_cfg_reg_a() {
    let reg = CfgRegA::default().into_bits();
    assert_eq!(reg, 0b00000011);
}

#[test]
fn test_default_cfg_reg_b() {
    let reg = CfgRegB::default().into_bits();
    assert_eq!(reg, 0b00000000);
}

#[test]
fn test_default_cfg_reg_c() {
    let reg = CfgRegC::default().into_bits();
    assert_eq!(reg, 0b00000000);
}

#[test]
fn test_default_int_ctrl_reg() {
    let reg = IntCrtlReg::default().into_bits();
    assert_eq!(reg, 0b11100000);
}

#[test]
fn test_default_int_ths_2_reg() {
    let reg = IntThs::default().into_bits();
    assert_eq!(reg, 0b0000000000000000);
}
