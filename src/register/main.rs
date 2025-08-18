use crate::Error;
use crate::Lis2mdl;
use bitfield_struct::bitfield;
use derive_more::TryFrom;
use st_mem_bank_macro::{named_register, register};
use st_mems_bus::BusOperation;

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum Reg {
    OffsetXRegL = 0x45,
    OffsetXRegH = 0x46,
    OffsetYRegL = 0x47,
    OffsetYRegH = 0x48,
    OffsetZRegL = 0x49,
    OffsetZRegH = 0x4A,
    WhoAmI = 0x4F,
    CfgRegA = 0x60,
    CfgRegB = 0x61,
    CfgRegC = 0x62,
    IntCrtlReg = 0x63,
    IntSourceReg = 0x64,
    IntThsLReg = 0x65,
    IntThsHReg = 0x66,
    StatusReg = 0x67,
    OutxLReg = 0x68,
    OutxHReg = 0x69,
    OutyLReg = 0x6A,
    OutyHReg = 0x6B,
    OutzLReg = 0x6C,
    OutzHReg = 0x6D,
    TempOutLReg = 0x6E,
    TempOutHReg = 0x6F,
}

/// Offset XYZ (0x45 - 0x4A)
///
/// Magnetic user offset for hard-iron to compensate environmental
/// effects.
/// 1LSB = 1.5mg
#[named_register(address = Reg::OffsetXRegL, access_type = Lis2mdl, generics = 1)]
pub struct OffsetXYZ {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// WHO_AM_I (0x4F)
///
/// The identification register is used to identify the device
#[register(address = Reg::WhoAmI, access_type = Lis2mdl, generics = 1)]
pub struct WhoAmI(pub u8);

/// CFG_REG_A (0x60)
///
/// Configuration register A (R/W)
/// Controls output data rate, power mode, reboot, soft reset, and temperature compensation.
#[register(address = Reg::CfgRegA, access_type = Lis2mdl, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct CfgRegA {
    /// Mode of operation selection (2 bits)
    /// 00: Continuous mode (continuous measurement, data-ready signal generated)
    /// 01: Single measurement mode (performs one measurement, sets DRDY high, then idle)
    /// 10: Idle mode (I2C and SPI active)
    /// 11: Idle mode (default)
    /// Connected enum: Md
    #[bits(2)]
    pub md: u8,
    /// Output data rate configuration (2 bits)
    /// 00: 10 Hz (default)
    /// 01: 20 Hz
    /// 10: 50 Hz
    /// 11: 100 Hz
    /// Connected enum: Odr
    #[bits(2)]
    pub odr: u8,
    /// Low-power mode enable (1 bit)
    /// 0: High-resolution mode
    /// 1: Low-power mode enabled
    /// Connected enum: Lp
    #[bits(1)]
    pub lp: u8,
    /// Soft reset configuration and user registers (1 bit)
    /// When set, resets configuration and user registers; flash registers keep their values.
    #[bits(1)]
    pub soft_rst: u8,
    /// Reboot magnetometer memory content (1 bit)
    /// When set, reboots magnetometer memory content.
    #[bits(1)]
    pub reboot: u8,
    /// Temperature compensation enable (1 bit)
    /// Must be set to 1 for correct operation.
    #[bits(1)]
    pub comp_temp_en: u8,
}

/// CFG_REG_B (0x61)
///
/// Configuration register B (R/W)
/// Controls offset cancellation, interrupt on data off, set pulse frequency, and low-pass filter.
#[register(address = Reg::CfgRegB, access_type = Lis2mdl, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct CfgRegB {
    /// Digital low-pass filter enable (1 bit)
    /// 0: Filter disabled (bandwidth = ODR/2)
    /// 1: Filter enabled (bandwidth = ODR/4)
    /// Connected enum: Lpf
    #[bits(1)]
    pub lpf: u8,
    /// Set pulse frequency selection (2 bits)
    /// 0: Set pulse released every 64 ODR cycles (default)
    /// 1: Set pulse released only at power-on after power-down condition
    /// Connected enum: SetRst
    #[bits(2)]
    pub set_rst: u8,
    /// Interrupt block checks data after hard-iron correction (1 bit)
    /// If 1, interrupt recognition checks data after hard-iron correction.
    /// Connected enum: IntOnDataoff
    #[bits(1)]
    pub int_on_dataoff: u8,
    /// Offset cancellation in single measurement mode enable (1 bit)
    /// 0: Disabled (default)
    /// 1: Enabled (OFF_CANC bit must be set to 1 when enabling)
    #[bits(1)]
    pub off_canc_one_shot: u8,
    /// Reserved bits (3 bits, read-only)
    #[bits(3, access = RO)]
    pub not_used_01: u8,
}

/// CFG_REG_C (0x62)
///
/// Configuration register C (R/W)
/// Controls interrupt pin behavior, interface selection, byte order, SPI mode, self-test, and data-ready pin.
#[register(address = Reg::CfgRegC, access_type = Lis2mdl, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct CfgRegC {
    /// Data-ready signal driven on INT/DRDY pin (push-pull output) (1 bit)
    #[bits(1)]
    pub drdy_on_pin: u8,
    /// Self-test enable (1 bit)
    #[bits(1)]
    pub self_test: u8,
    /// Enable 4-wire SPI interface (disables interrupt and data-ready signaling) (1 bit)
    /// Connected enum: Sim
    #[bits(1)]
    pub four_wire_spi: u8,
    /// Byte order inversion (low and high bytes of data are swapped) (1 bit)
    #[bits(1)]
    pub ble: u8,
    /// Block data update enable (prevents reading incoherent data during update) (1 bit)
    /// Connected enum: Ble
    #[bits(1)]
    pub bdu: u8,
    /// I2C interface disable (1 bit)
    /// If 1, I2C interface is inhibited; only SPI interface can be used.
    /// Connected enum: I2cDis
    #[bits(1)]
    pub i2c_dis: u8,
    /// Interrupt signal driven on INT/DRDY pin (push-pull output) (1 bit)
    #[bits(1)]
    pub int_on_pin: u8,
    /// Reserved bit (1 bit, read-only)
    #[bits(1, access = RO)]
    pub not_used_02: u8,
}

/// INT_CTRL_REG (0x63)
///
/// Interrupt control register (R/W)
/// Enables and configures interrupt recognition.
#[register(address = Reg::IntCrtlReg, access_type = Lis2mdl, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct IntCrtlReg {
    /// Interrupt enable (1 bit)
    /// Enables generation of interrupt (INT bit in INT_SOURCE_REG).
    #[bits(1)]
    pub ien: u8,
    /// Interrupt latch enable (1 bit)
    /// 0: INT bit is pulsed
    /// 1: INT bit is latched until INT_SOURCE_REG is read
    #[bits(1)]
    pub iel: u8,
    /// Interrupt polarity (1 bit)
    /// 0: INT = 0 signals interrupt
    /// 1: INT = 1 signals interrupt
    #[bits(1)]
    pub iea: u8,
    /// Reserved bits (2 bits, read-only)
    #[bits(2, access = RO)]
    pub not_used_01: u8,
    /// Z-axis interrupt enable (1 bit)
    #[bits(1)]
    pub zien: u8,
    /// Y-axis interrupt enable (1 bit)
    #[bits(1)]
    pub yien: u8,
    /// X-axis interrupt enable (1 bit)
    #[bits(1)]
    pub xien: u8,
}

/// INT_SOURCE_REG (0x64)
///
/// Interrupt source register (R)
/// Indicates interrupt event status and axis threshold crossing.
#[register(address = Reg::IntSourceReg, access_type = Lis2mdl, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct IntSourceReg {
    /// Interrupt event occurred (1 bit)
    #[bits(1)]
    pub _int: u8,
    /// Magnetic ROI flag (1 bit)
    /// Always enabled; reset by reading this register.
    #[bits(1)]
    pub mroi: u8,
    /// Negative threshold exceeded on Z-axis (1 bit)
    #[bits(1)]
    pub n_th_s_z: u8,
    /// Negative threshold exceeded on Y-axis (1 bit)
    #[bits(1)]
    pub n_th_s_y: u8,
    /// Negative threshold exceeded on X-axis (1 bit)
    #[bits(1)]
    pub n_th_s_x: u8,
    /// Positive threshold exceeded on Z-axis (1 bit)
    #[bits(1)]
    pub p_th_s_z: u8,
    /// Positive threshold exceeded on Y-axis (1 bit)
    #[bits(1)]
    pub p_th_s_y: u8,
    /// Positive threshold exceeded on X-axis (1 bit)
    #[bits(1)]
    pub p_th_s_x: u8,
}

/// INT_THS_L_REG - INT_THS_H_REG (0x65 - 0x66)
///
/// These registers set the threshold value for the output to generate the interrupt (INT bit in INT_SOURCE_REG(64h)).
/// This threshold is common to all three (axes) output values and is unsigned unipolar. The threshold value is correlated
/// to the current gain and it is unsigned because the threshold is considered as an absolute value, but crossing the
/// threshold is detected for both the positive and negative sides.
#[register(address = Reg::IntThsLReg, access_type = Lis2mdl, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct IntThs {
    #[bits(16)]
    pub threshold: u16,
}

/// STATUS_REG (0x67)
///
/// Status register (R)
/// Indicates data availability and overrun status for each axis.
#[register(address = Reg::StatusReg, access_type = Lis2mdl, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct StatusReg {
    /// X-axis new data available (1 bit)
    #[bits(1)]
    pub xda: u8,
    /// Y-axis new data available (1 bit)
    #[bits(1)]
    pub yda: u8,
    /// Z-axis new data available (1 bit)
    #[bits(1)]
    pub zda: u8,
    /// X-, Y-, and Z-axis new data available (1 bit)
    #[bits(1)]
    pub zyxda: u8,
    /// X-axis data overrun (1 bit)
    #[bits(1)]
    pub _xor: u8,
    /// Y-axis data overrun (1 bit)
    #[bits(1)]
    pub yor: u8,
    /// Z-axis data overrun (1 bit)
    #[bits(1)]
    pub zor: u8,
    /// X-, Y-, and Z-axis data overrun (1 bit)
    #[bits(1)]
    pub zyxor: u8,
}

/// OutXYZ (0x68 - 0x6D)
///
/// The output data represents the raw magnetic data only if
/// relative OFFSET_axis_REG is equal to zero, otherwise hard-iron
/// calibration is included.
#[named_register(address = Reg::OutxLReg, access_type = Lis2mdl, generics = 1)]
pub struct OutXYZ {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// TempOut (0x6E - 0x6F)
///
/// Temperature register of the sensor
#[register(address = Reg::TempOutLReg, access_type = Lis2mdl, generics = 1)]
pub struct TempOut(pub i16);

/// Mode of operation
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default)]
pub enum Md {
    /// Continuous mode (continuous measurement, data-ready signal generated)
    ContinuousMode = 0,
    /// Single measurement mode (performs one measurement, sets DRDY high, then idle)
    SingleTrigger = 1,
    /// Idle mode (I2C and SPI active): bits 10 or 11
    #[default]
    PowerDown = 3,
}

impl TryFrom<u8> for Md {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        let md = match value {
            0 => Md::ContinuousMode,
            1 => Md::SingleTrigger,
            _ => Md::PowerDown,
        };

        Ok(md)
    }
}

/// Output data rate configuration for the sensor.
///
/// Determines the frequency at which data is updated and made available.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum Odr {
    /// Output data rate at 10 Hz (default)
    #[default]
    _10hz = 0,
    /// Output data rate at 20 Hz
    _20hz = 1,
    /// Output data rate at 50 Hz
    _50hz = 2,
    /// Output data rate at 100 Hz
    _100hz = 3,
}

/// Power mode
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum PowerMode {
    /// High-resolution mode
    #[default]
    HighResolution = 0,
    /// Low-power mode enabled
    LowPower = 1,
}

/// Digital low pass filter
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum LowPassFilter {
    /// Filter disabled (bandwidth = ODR/2)
    #[default]
    OdrDiv2 = 0,
    /// Filter enabled (bandwidth = ODR/4)
    OdrDiv4 = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum SetRst {
    /// Set pulse released every 64 ODR cycles (default)
    #[default]
    SetSensOdrDiv63 = 0,
    /// Set pulse released only at power-on after power-down condition
    SensOffCancEveryOdr = 1,
    /// Set pulse released only at power-on after power-down condition
    SetSensOnlyAtPowerOn = 2,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum Ble {
    /// LSB at low address (default byte order)
    #[default]
    LsbAtLowAdd = 0,
    /// MSB at low address (byte order inverted)
    MsbAtLowAdd = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum IntOnDataoff {
    /// Interrupt recognition checks data before hard-iron correction
    #[default]
    CheckBefore = 0,
    /// Interrupt recognition checks data after hard-iron correction
    CheckAfter = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum Sim {
    /// SPI 4-wire interface enabled (disables interrupt and data-ready signaling)
    Spi4Wire = 1,
    /// SPI 3-wire interface enabled
    #[default]
    Spi3Wire = 0,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum I2cSwitch {
    /// I2C interface enabled (default)
    #[default]
    I2cEnable = 0,
    /// I2C interface disabled; only SPI interface can be used
    I2cDisable = 1,
}
