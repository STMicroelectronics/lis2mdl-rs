#![no_std]
#![doc = include_str!("../README.md")]
pub mod prelude;
pub mod register;

use core::fmt::Debug;
use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::spi::SpiDevice;
use prelude::*;
use st_mems_bus::*;

/// Driver for LIS2MDL sensor.
///
/// The struct takes a bus object to write to the registers.
/// The bus is generalized over the BusOperation trait, allowing the use
/// of I2C or SPI protocols; this also allows the user to implement sharing
/// techniques to share the underlying bus.
pub struct Lis2mdl<B> {
    /// The bus driver.
    pub bus: B,
}

/// Driver errors.
#[derive(Debug)]
pub enum Error<B> {
    Bus(B),          // Error at the bus level
    UnexpectedValue, // Unexpected value read from a register
}
impl<B> Lis2mdl<B>
where
    B: BusOperation,
{
    /// Constructor method using a generic Bus that implements BusOperation
    pub fn new_bus(bus: B) -> Self {
        Self { bus }
    }
}
impl<P> Lis2mdl<i2c::I2cBus<P>>
where
    P: I2c,
{
    /// Constructor method for using the I2C bus.
    pub fn new_i2c(i2c: P, address: I2CAddress) -> Self {
        // Initialize the I2C bus with the COMPONENT address
        let bus = i2c::I2cBus::new(i2c, address as SevenBitAddress);
        Self { bus }
    }
}
impl<P> Lis2mdl<spi::SpiBus<P>>
where
    P: SpiDevice,
{
    /// Constructor method for using the SPI bus.
    pub fn new_spi(spi: P) -> Self {
        // Initialize the SPI bus
        let bus = spi::SpiBus::new(spi);
        Self { bus }
    }
}

impl<B: BusOperation> Lis2mdl<B> {
    pub fn write_to_register(&mut self, reg: u8, buf: &[u8]) -> Result<(), Error<B::Error>> {
        self.bus.write_to_register(reg, buf).map_err(Error::Bus)
    }

    pub fn read_from_register(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<B::Error>> {
        self.bus.read_from_register(reg, buf).map_err(Error::Bus)
    }

    /// Set magnetic user offset for hard-iron to compensate environmental effects.
    ///
    /// Data format is the same of output data raw: two's complement
    /// with 1LSb = 1.5mg. These values act on the magnetic output data
    /// value in order to delete the environmental offset.
    pub fn mag_user_offset_set(&mut self, val: &[i16; 3]) -> Result<(), Error<B::Error>> {
        let offset = OffsetXYZ {
            x: val[0],
            y: val[1],
            z: val[2],
        };
        offset.write(self)?;

        Ok(())
    }

    /// Get magnetic user offset for hard-iron to compensate environmental effects.
    ///
    /// Data format is the same of output data raw: two's complement
    /// with 1LSb = 1.5mg. These values act on the magnetic output data
    /// value in order to delete the environmental offset.
    pub fn mag_user_offset_get(&mut self) -> Result<[i16; 3], Error<B::Error>> {
        let val = OffsetXYZ::read(self)?;

        Ok([val.x, val.y, val.z])
    }

    /// Set Operating mode.
    pub fn operating_mode_set(&mut self, val: Md) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegA::read(self)?;
        reg.set_md(val as u8);
        reg.write(self)
    }
    /// Get Operating mode.
    pub fn operating_mode_get(&mut self) -> Result<Md, Error<B::Error>> {
        let reg = CfgRegA::read(self)?;

        let val = Md::try_from(reg.md()).unwrap_or_default();
        Ok(val)
    }
    /// Set Output data rate(ODR).
    pub fn data_rate_set(&mut self, val: Odr) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegA::read(self)?;
        reg.set_odr(val as u8);
        reg.write(self)
    }
    /// Get Output data rate(ODR).
    pub fn data_rate_get(&mut self) -> Result<Odr, Error<B::Error>> {
        let reg = CfgRegA::read(self)?;
        let val = Odr::try_from(reg.odr()).unwrap_or_default();
        Ok(val)
    }
    /// Enables/Disable high-resolution/low-power mode.
    pub fn power_mode_set(&mut self, val: PowerMode) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegA::read(self)?;
        reg.set_lp(val as u8);
        reg.write(self)?;

        Ok(())
    }
    /// Get high-resolution/low-power mode.
    pub fn power_mode_get(&mut self) -> Result<PowerMode, Error<B::Error>> {
        let reg = CfgRegA::read(self)?;

        let val = PowerMode::try_from(reg.lp()).unwrap_or_default();
        Ok(val)
    }
    /// Enables/Disables the magnetometer temperature compensation.
    pub fn offset_temp_comp_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegA::read(self)?;
        reg.set_comp_temp_en(val);
        reg.write(self)?;

        Ok(())
    }
    /// Get configuration (enable/disable) for magnetometer temperature compensation.
    pub fn offset_temp_comp_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CfgRegA::read(self)?.comp_temp_en();

        Ok(val)
    }
    /// Set Low-pass bandwidth selection.
    pub fn low_pass_bandwidth_set(&mut self, val: LowPassFilter) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegB::read(self)?;
        reg.set_lpf(val as u8);
        reg.write(self)?;

        Ok(())
    }
    /// Get Low-pass bandwidth.
    pub fn low_pass_bandwidth_get(&mut self) -> Result<LowPassFilter, Error<B::Error>> {
        let reg = CfgRegB::read(self)?;

        let val = LowPassFilter::try_from(reg.lpf()).unwrap_or_default();
        Ok(val)
    }
    /// Set Reset mode.
    pub fn set_rst_mode_set(&mut self, val: SetRst) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegB::read(self)?;
        reg.set_set_rst(val as u8);
        reg.write(self)?;

        Ok(())
    }
    /// Get Reset mode.
    pub fn set_rst_mode_get(&mut self) -> Result<SetRst, Error<B::Error>> {
        let reg = CfgRegB::read(self)?;

        let val = SetRst::try_from(reg.set_rst()).unwrap_or_default();
        Ok(val)
    }
    /// Enables/Disables offset cancellation in single measurement mode.
    ///
    /// The OFF_CANC bit must be set to 1 when enabling offset
    /// cancellation in single measurement mode, which means a
    /// call function: set_rst_mode(SENS_OFF_CANC_EVERY_ODR)
    /// is needed.
    pub fn set_rst_sensor_single_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegB::read(self)?;
        reg.set_off_canc_one_shot(val);
        reg.write(self)?;

        Ok(())
    }
    /// Get configuration (enable/disable) for offset cancellation in single measurement mode.
    ///
    /// The OFF_CANC bit must be set to 1 when enabling offset
    /// cancellation in single measurement mode. This means calling
    /// the function set_rst_mode(SensOffCancEveryOdr) is needed.
    pub fn set_rst_sensor_single_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CfgRegB::read(self)?.off_canc_one_shot();

        Ok(val)
    }
    /// Enable/Disable Block data update.
    ///
    /// If val is 1: BDU is active
    pub fn block_data_update_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegC::read(self)?;
        reg.set_bdu(val);
        reg.write(self)?;

        Ok(())
    }
    /// Get Block data update configuration (enable/disable).
    pub fn block_data_update_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CfgRegC::read(self)?.bdu();

        Ok(val)
    }
    /// Get magnetic data ready data available.
    ///
    /// If event available returns 1.
    pub fn mag_data_ready_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = StatusReg::read(self)?.zyxda();

        Ok(val)
    }
    /// Get magnetic data overrun event.
    ///
    /// 0: no overrun has occurred. 1: a new set of data has overwritten the previous set
    pub fn mag_data_ovr_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = StatusReg::read(self)?.zyxor();

        Ok(val)
    }
    /// Get Magnetic raw value.
    ///
    /// Need conversion using `from_lsb_to_mgauss`
    pub fn magnetic_raw_get(&mut self) -> Result<[i16; 3], Error<B::Error>> {
        let val = OutXYZ::read(self)?;

        Ok([val.x, val.y, val.z])
    }

    /// Get temperature raw value.
    pub fn temperature_raw_get(&mut self) -> Result<i16, Error<B::Error>> {
        TempOut::read(self).map(|temp| temp.0)
    }

    /// Get Device Id.
    ///
    /// Reads the WHO_AM_I register.
    pub fn device_id_get(&mut self) -> Result<u8, Error<B::Error>> {
        WhoAmI::read(self).map(|who_am_i| who_am_i.0)
    }
    /// Set Software reset.
    ///
    /// If val is 1: Restore the default values in user registers.
    /// Flash register keep their values.
    pub fn reset_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegA::read(self)?;
        reg.set_soft_rst(val);
        reg.write(self)?;

        Ok(())
    }
    /// Get Software reset configuration.
    ///
    /// If set to 1: Restore the default values in user registers.
    pub fn reset_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CfgRegA::read(self)?.soft_rst();

        Ok(val)
    }
    /// Set Reboot memory content.
    ///
    /// If val is 1: Reload the calibration parameters.
    pub fn boot_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegA::read(self)?;
        reg.set_reboot(val);
        reg.write(self)?;

        Ok(())
    }
    /// Reboot memory content. Reload the calibration parameters.
    pub fn boot_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CfgRegA::read(self)?.reboot();

        Ok(val)
    }
    /// Set Selftest mode.
    pub fn self_test_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegC::read(self)?;
        reg.set_self_test(val);
        reg.write(self)?;

        Ok(())
    }
    /// Get Selftest configuration.
    pub fn self_test_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CfgRegC::read(self)?.self_test();

        Ok(val)
    }
    /// Set Big/Little Endian data format.
    pub fn data_format_set(&mut self, val: Ble) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegC::read(self)?;
        reg.set_ble(val as u8);
        reg.write(self)?;

        Ok(())
    }
    /// Get Big/Little Endian data format.
    pub fn data_format_get(&mut self) -> Result<Ble, Error<B::Error>> {
        let reg = CfgRegC::read(self)?;

        let val = Ble::try_from(reg.ble()).unwrap_or_default();
        Ok(val)
    }
    /// Get info about device status.
    pub fn status_get(&mut self) -> Result<StatusReg, Error<B::Error>> {
        let val = StatusReg::read(self)?;

        Ok(val)
    }
    /// Set the interrupt block recognition checks data after/before the
    /// hard-iron correction to discover the interrupt.
    pub fn offset_int_conf_set(&mut self, val: IntOnDataoff) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegB::read(self)?;
        reg.set_int_on_dataoff(val as u8);
        reg.write(self)?;

        Ok(())
    }
    /// Retrive the interrupt block recognition checks data after/before the
    /// hard-iron correction to discover the interrupt.
    pub fn offset_int_conf_get(&mut self) -> Result<IntOnDataoff, Error<B::Error>> {
        let reg = CfgRegB::read(self)?;

        let val = IntOnDataoff::try_from(reg.int_on_dataoff()).unwrap_or_default();
        Ok(val)
    }
    /// Set Data-ready signal on INT_DRDY pin.
    pub fn drdy_on_pin_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegC::read(self)?;
        reg.set_drdy_on_pin(val);
        reg.write(self)?;

        Ok(())
    }
    /// Get Data-ready signal on INT_DRDY pin.
    pub fn drdy_on_pin_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CfgRegC::read(self)?.drdy_on_pin();

        Ok(val)
    }
    /// Set Interrupt signal on INT_DRDY pin.
    pub fn int_on_pin_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegC::read(self)?;
        reg.set_int_on_pin(val);
        reg.write(self)?;

        Ok(())
    }
    /// Get Interrupt signal on INT_DRDY pin.
    pub fn int_on_pin_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CfgRegC::read(self)?.int_on_pin();

        Ok(val)
    }
    /// Set Interrupt generator configuration register.
    pub fn int_gen_conf_set(&mut self, val: &IntCrtlReg) -> Result<(), Error<B::Error>> {
        val.write(self)
    }
    /// Get Interrupt generator configuration register.
    pub fn int_gen_conf_get(&mut self) -> Result<IntCrtlReg, Error<B::Error>> {
        IntCrtlReg::read(self)
    }
    /// Get Interrupt generator source register.
    pub fn int_gen_source_get(&mut self) -> Result<IntSourceReg, Error<B::Error>> {
        IntSourceReg::read(self)
    }
    /// Set User-defined threshold value for xl interrupt event on generator.
    ///
    /// Data format is the same as output data raw: two's complement with
    /// 1LSb = 1.5mg.
    pub fn int_gen_threshold_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        IntThs::new().with_threshold(val).write(self)
    }
    /// Get User-defined threshold value for xl interrupt event on generator.
    ///
    /// Data format is the same as output data raw: two�s complement with
    /// 1LSb = 1.5mg.
    pub fn int_gen_threshold_get(&mut self) -> Result<u16, Error<B::Error>> {
        Ok(IntThs::read(self)?.threshold())
    }
    /// Set SPI Serial Interface Mode.
    ///
    /// If Sim::Spi4Wire enable SDO line on pin 7.
    pub fn spi_mode_set(&mut self, val: Sim) -> Result<(), Error<B::Error>> {
        let mut reg_val = CfgRegC::read(self)?;
        reg_val.set_four_wire_spi(val as u8);
        reg_val.write(self)?;

        Ok(())
    }

    /// Get SPI Serial Interface Mode.
    pub fn spi_mode_get(&mut self) -> Result<Sim, Error<B::Error>> {
        let reg_val = CfgRegC::read(self)?;

        let val = Sim::try_from(reg_val.four_wire_spi()).unwrap_or_default();
        Ok(val)
    }

    /// Disable/Enable I2C interface.
    ///
    /// As default I2C interface is enabled, use this function to switch if needed
    pub fn i2c_interface_set(&mut self, val: I2cSwitch) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegC::read(self)?;
        reg.set_i2c_dis(val as u8);
        reg.write(self)?;

        Ok(())
    }
    /// Get I2C interface configuration (enable/disable).
    pub fn i2c_interface_get(&mut self) -> Result<I2cSwitch, Error<B::Error>> {
        let reg = CfgRegC::read(self)?;

        let val = I2cSwitch::try_from(reg.i2c_dis()).unwrap_or_default();
        Ok(val)
    }
}

pub fn from_lsb_to_mgauss(lsb: i16) -> f32 {
    lsb as f32 * 1.5
}

pub fn from_lsb_to_celsius(lsb: i16) -> f32 {
    lsb as f32 / 8.0 + 25.0
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum I2CAddress {
    I2cAdd = 0x1E,
}

pub const ID: u8 = 0x40;
